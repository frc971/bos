import gdb

class UnitsPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        try:
            # Most WPILib units store value in m_value
            value = self.val['m_value']

            typename = str(self.val.type.strip_typedefs())

            # Try to extract unit name from template
            # Example: units::unit_t<units::meters, double>
            unit_name = "unit"
            if "<" in typename and ">" in typename:
                inside = typename.split("<", 1)[1].rsplit(">", 1)[0]
                unit_name = inside.split(",")[0].strip()

                # Clean up common namespace noise
                unit_name = unit_name.replace("units::", "")

            return f"{value} {unit_name}"
        except Exception as e:
            return f"<units error: {e}>"

class Pose3dPrinter:
    def __init__(self, val):
        self.val = val

    def _get_unit_value(self, unit_val):
        """Extract double from WPILib unit_t safely"""
        try:
            return float(unit_val['m_value'])
        except:
            try:
                # fallback if GDB is weird about templates
                return float(unit_val.cast(unit_val.type)['m_value'])
            except:
                return None

    def _fmt(self, v, unit=""):
        if v is None:
            return "?"
        return f"{v:.3f}{unit}"

    def to_string(self):
        try:
            t = self.val['m_translation']

            x = self._get_unit_value(t['m_x'])
            y = self._get_unit_value(t['m_y'])
            z = self._get_unit_value(t['m_z'])

            r = self.val['m_rotation']['m_q']

            w = float(r['m_w'])
            xq = float(r['m_x'])
            yq = float(r['m_y'])
            zq = float(r['m_z'])

            return (
                f"Pose3d("
                f"x={self._fmt(x,'m')}, "
                f"y={self._fmt(y,'m')}, "
                f"z={self._fmt(z,'m')}, "
                f"quat=[{w:.3f}, {xq:.3f}, {yq:.3f}, {zq:.3f}]"
                f")"
            )

        except Exception as e:
            return f"Pose3d(<error: {e}>)"

class PositionEstimatePrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        try:
            return self.val['pose']
        except Exception as e:
            return f"Pose3d(<error: {e}>)"


def pose3d_lookup_function(val):
    typename = str(val.type.strip_typedefs())

    if "frc::Pose3d" in typename:
        return Pose3dPrinter(val)

    if "units::unit_t" in typename:
        return UnitsPrinter(val)

    if "localization::position_estimate_t" in typename:
        return PositionEstimatePrinter(val)

    return None


def register_printers(objfile):
    gdb.pretty_printers.append(pose3d_lookup_function)


register_printers(gdb.current_objfile())
