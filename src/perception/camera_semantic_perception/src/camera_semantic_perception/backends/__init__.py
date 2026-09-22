"""Semantic inference backend registry."""


def create_backend(name, **kwargs):
    normalized = str(name).strip().lower()
    if normalized == "twinlite":
        from .twinlite import TwinLiteBackend

        return TwinLiteBackend(**kwargs)
    raise ValueError("unsupported semantic backend: {}".format(name))


__all__ = ["create_backend"]
