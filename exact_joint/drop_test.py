"""Analytic release-to-contact kinematics, NOT an impact or survival solver."""

import dataclasses
import math


@dataclasses.dataclass(frozen=True)
class DropCase:
    """SI example assumptions for an upright, unloaded whole-leg release."""

    height_m: float = 0.070
    upper_mass_kg: float = 0.030
    lower_mass_kg: float = 0.020
    gravity_m_s2: float = 9.81
    playback_speed: float = 0.05

    def __post_init__(self):
        values = dataclasses.asdict(self).values()
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("Drop inputs must be finite and positive (SI units).")
        if self.playback_speed > 1:
            raise ValueError("Preview playback speed must be at most 1x.")

    @property
    def mass_kg(self):
        return self.upper_mass_kg + self.lower_mass_kg

    @property
    def contact_time_s(self):
        return math.sqrt(2 * self.height_m / self.gravity_m_s2)

    def sample(self, time_s):
        """Clamp at first contact; velocity is the incident value, not a rebound."""
        if not math.isfinite(time_s) or time_s < 0:
            raise ValueError("Elapsed physical time must be finite and nonnegative.")
        elapsed = min(time_s, self.contact_time_s)
        distance = min(0.5 * self.gravity_m_s2 * elapsed**2, self.height_m)
        velocity = self.gravity_m_s2 * elapsed
        return {
            "time_s": elapsed,
            "clearance_m": max(0.0, self.height_m - distance),
            "fallen_m": distance,
            "incident_downward_speed_m_s": velocity,
            "kinetic_energy_j": 0.5 * self.mass_kg * velocity**2,
            "at_first_contact": time_s >= self.contact_time_s,
            "impact_force_n": None,
            "impact_stress_pa": None,
            "survives": None,
        }

    def report(self):
        return {
            "scope": "Analytic free fall to first contact only; no impact FEM",
            "assumptions_si": dataclasses.asdict(self),
            "total_mass_kg": self.mass_kg,
            "mass_provenance": "Provisional example authorized by user; includes knee and plates, no payload",
            "orientation": "Upright, foot first; zero initial velocity and zero cable tension",
            "surface": "Rigid horizontal plane; only first-contact height is evaluated",
            "height_definition": "Initial lowest foot point to floor top, not center-of-mass height",
            "contact": self.sample(self.contact_time_s),
            "mass_sensitivity": [
                {"mass_kg": mass, "incident_energy_j": mass * self.gravity_m_s2 * self.height_m}
                for mass in (0.025, 0.050, 0.100)
            ],
            "status": "SURVIVAL_INDETERMINATE",
            "blockers": [
                "Quasistatic FEM has no mass/inertia or transient impact integration",
                "No foot contact, self-contact, impact damping or restitution model",
                "No calibrated rate-dependent PLA/PET failure or adhesive separation law",
                "Mesh response is unconverged; PET reference volume changes with refinement",
                "Measured mass distribution, material coupons and impact data are unavailable",
            ],
        }
