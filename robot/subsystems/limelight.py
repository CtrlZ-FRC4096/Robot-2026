import math
from limelight import Limelight
import const

detector_idx = 3
april_tags_idx = 1
retroreflective_idx = 2


class Limelight_Wrapper(Limelight):
    ll_mount_angle = 19  # different from siren pls measure these
    ll_height = 9.75  # different from siren pls measure these
    ll_yaw = 0  # different from siren pls measure these
    ll_offset = 0  # not measured
    offset_from_shaft = 3  # different from siren pls measure these

    def set_to_detector(self):
        self.pipeline_index = detector_idx

    def set_to_april_tags(self):
        self.pipeline_index = april_tags_idx

    def set_to_retroreflective(self):
        self.pipeline_index = retroreflective_idx

    @property
    def angle_to_nearest_algae(self) -> float | None:
        results = self.latest_results
        if not results:
            return None
        results = results.detector_results
        results = [x for x in results if x.class_name == "algae"] # Will need to change this to the correct class name
        if not results or not self.pipeline_index == 3:
            return None
        nearest = max(results, key=lambda x: x.ta)
        return nearest.tx
