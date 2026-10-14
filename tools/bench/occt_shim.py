"""Make `import cadquery` work without VTK (only the tessellation-to-VTK helpers need it)."""
import OCP.IVtkOCC as _m


class _Stub:
    pass


for _n in ("IVtkOCC_Shape", "IVtkOCC_ShapeMesher"):
    if not hasattr(_m, _n):
        setattr(_m, _n, _Stub)
try:
    import OCP.IVtkVTK as _v
    for _n in ("IVtkVTK_ShapeData",):
        if not hasattr(_v, _n):
            setattr(_v, _n, _Stub)
except Exception:
    pass
import cadquery as cq  # noqa: E402,F401
