"""Use a libcurl easy handle across one mission to reuse authenticated HTTPS."""

import ctypes as C
from ctypes.util import find_library
from functools import cache
import json
import threading


# Stable public libcurl ABI, using its long, object and function option types.
URL, BODY, HEADERS = 10002, 10015, 10023
BODY_SIZE, HTTP_VERSION, NO_SIGNAL, TIMEOUT_MS, PROTOCOLS = 60, 84, 99, 155, 181
WRITE = 20011
LONG_INFO, DOUBLE_INFO = 0x200000, 0x300000
WRITE_FUNCTION = C.CFUNCTYPE(C.c_size_t, C.c_void_p, C.c_size_t, C.c_size_t, C.c_void_p)


@cache
def load_curl():
    path = find_library("curl")
    if path is None:
        raise RuntimeError("the Luna mission requires libcurl")
    library = C.CDLL(path)
    prototypes = {
        "curl_global_init": (C.c_int, [C.c_long]),
        "curl_easy_init": (C.c_void_p, []),
        "curl_easy_reset": (None, [C.c_void_p]),
        "curl_easy_cleanup": (None, [C.c_void_p]),
        "curl_easy_perform": (C.c_int, [C.c_void_p]),
        "curl_easy_setopt": (C.c_int, [C.c_void_p, C.c_int]),
        "curl_easy_getinfo": (C.c_int, [C.c_void_p, C.c_int]),
        "curl_slist_append": (C.c_void_p, [C.c_void_p, C.c_char_p]),
        "curl_slist_free_all": (None, [C.c_void_p]),
    }
    for name, (returns, arguments) in prototypes.items():
        function = getattr(library, name)
        function.restype, function.argtypes = returns, arguments
    if library.curl_global_init(3):
        raise RuntimeError("libcurl initialization failed")
    return library


class PooledHTTPS:
    def __init__(self):
        self.curl = load_curl()
        self.handle = self.curl.curl_easy_init()
        self.lock = threading.Lock()
        if not self.handle:
            raise RuntimeError("libcurl could not allocate a session")

    def __enter__(self):
        return self

    def __exit__(self, *_):
        with self.lock:
            if self.handle:
                self.curl.curl_easy_cleanup(self.handle)
                self.handle = None

    def __call__(self, payload, key, timeout):
        from navigation.luna_probe import ProbeError, decode_response

        with self.lock:
            if not self.handle:
                raise RuntimeError("Luna HTTPS session is closed")
            if not key or any(c not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_" for c in key):
                raise ProbeError("invalid_credential_format")
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            chunks = bytearray()
            handle, curl = self.handle, self.curl
            headers = None

            @WRITE_FUNCTION
            def receive(pointer, size, count, _):
                length = size * count
                if len(chunks) + length > 4_194_304:
                    return 0
                chunks.extend(C.string_at(pointer, length))
                return length

            def option(code, value):
                if curl.curl_easy_setopt(handle, code, value):
                    raise ProbeError(f"transport_option_failed_{code}")

            def info(code, kind):
                value = kind()
                if curl.curl_easy_getinfo(handle, code, C.byref(value)):
                    raise ProbeError(f"transport_metric_failed_{code}")
                return value.value

            try:
                for header in ("Authorization: Bearer " + key, "Content-Type: application/json"):
                    appended = curl.curl_slist_append(headers, header.encode("ascii"))
                    if not appended:
                        raise ProbeError("transport_header_allocation")
                    headers = appended
                option(URL, C.c_char_p(b"https://api.openai.com/v1/responses"))
                option(HEADERS, C.c_void_p(headers))
                option(BODY, C.c_char_p(body))
                option(BODY_SIZE, C.c_long(len(body)))
                option(WRITE, receive)
                option(NO_SIGNAL, C.c_long(1))
                option(TIMEOUT_MS, C.c_long(max(1, round(timeout * 1000))))
                option(HTTP_VERSION, C.c_long(4))  # Prefer HTTP/2 over TLS.
                option(PROTOCOLS, C.c_long(2))  # HTTPS only.
                result = curl.curl_easy_perform(handle)
                if result:
                    raise ProbeError(f"transport_curl_{result}")
                status = info(LONG_INFO + 2, C.c_long)
                total = info(DOUBLE_INFO + 3, C.c_double)
                setup = info(DOUBLE_INFO + 6, C.c_double)
                timing = {"http_status": status, "total_seconds": total,
                          "connection_setup_seconds": setup, "uploaded_bytes": len(body),
                          "elapsed_after_connect_seconds": max(0.0, total - setup),
                          "new_connections": info(LONG_INFO + 26, C.c_long),
                          "transport": "pooled_libcurl"}
                return decode_response(chunks, status), timing
            finally:
                # Reset request pointers and credentials while retaining the connection cache.
                curl.curl_easy_reset(handle)
                if headers:
                    curl.curl_slist_free_all(headers)
