"""Encode the explicitly selected wire image and parse client-side HTTP timings."""

from io import BytesIO


def image_payload(image, profile):
    if profile == "standard":
        return image, "high"
    if profile != "lean":
        raise ValueError("unknown vision profile")
    from PIL import Image

    with Image.open(BytesIO(image)) as source:
        rgb = source.convert("RGB")
        rgb.thumbnail((640, 640), Image.Resampling.LANCZOS)
        buffer = BytesIO()
        rgb.quantize(colors=128).save(buffer, format="PNG")
        return buffer.getvalue(), "low"


def exchange_stats(stdout):
    body, timing = stdout.rsplit("\n", 1)
    status, dns, connection, tls, ready, first, end, uploaded = timing.split()
    dns, connection, tls, ready, first, end = map(float, (dns, connection, tls, ready, first, end))
    return body, {
        "http_status": int(status), "uploaded_bytes": int(uploaded),
        "dns_seconds": dns, "tcp_connected_seconds": connection, "tls_connected_seconds": tls,
        "connection_setup_seconds": ready, "first_byte_seconds": first, "total_seconds": end,
        "elapsed_after_connect_seconds": max(0.0, end - ready),
    }
