import json
import sys
from pathlib import Path as FilePath
from dataclasses import dataclass
from typing import List, Optional, Any, Callable

import wpilib
from wpimath.geometry import Translation2d, Rotation2d

# Adjust this import based on your exact file structure
from path import (
    Path, PathElement, TranslationTarget, RotationTarget,
    EventTrigger, Waypoint, PathConstraints,
    DefaultGlobalConstraints, RangedConstraint
)

def _resolve_project_root() -> FilePath:
    """Helper to initialize the class constant safely."""
    # try:
    return FilePath(wpilib.getDeployDirectory())
    # except Exception:
    #     # Allows unit tests or desktop environments
    #     return FilePath("src/main/deploy/autos")


class JsonUtils:
    """
    Utility class for loading and parsing path data from JSON files.
    """

    # ---------------------------------------------------------
    # Nested Classes & Constants
    # ---------------------------------------------------------

    @dataclass
    class ParsedPathComponents:
        """Container for parsed path components without constructing a full Path object."""
        elements: List[PathElement]
        constraints: PathConstraints
        default_global_constraints: DefaultGlobalConstraints

        def to_path(self) -> Path:
            return Path(self.elements, self.constraints, self.default_global_constraints)

    PROJECT_ROOT = _resolve_project_root()

    PATH_CONSTRAINT_KEY_ALIASES = [
        ["max_velocity_meters_per_sec"],
        ["max_acceleration_meters_per_sec2"],
        ["max_velocity_deg_per_sec"],
        ["max_acceleration_deg_per_sec2"],
        ["end_translation_tolerance_meters"],
        ["end_rotation_tolerance_deg"]
    ]

    GLOBAL_CONSTRAINT_KEY_ALIASES = [
        ["default_max_velocity_meters_per_sec", "max_velocity_meters_per_sec"],
        ["default_max_acceleration_meters_per_sec2", "max_acceleration_meters_per_sec2"],
        ["default_max_velocity_deg_per_sec", "max_velocity_deg_per_sec"],
        ["default_max_acceleration_deg_per_sec2", "max_acceleration_deg_per_sec2"],
        ["default_end_translation_tolerance_meters", "end_translation_tolerance_meters"],
        ["default_end_rotation_tolerance_deg", "end_rotation_tolerance_deg"],
        ["default_intermediate_handoff_radius_meters", "intermediate_handoff_radius_meters"]
    ]

    FALLBACK_GLOBAL_CONSTRAINTS = DefaultGlobalConstraints(
        4.0, 4.5, 540.0, 720.0, 0.05, 4.0, 0.2
    )

    # ---------------------------------------------------------
    # Public API (Static Methods)
    # ---------------------------------------------------------

    @staticmethod
    def load_path(path_file_name: str) -> Path:
        try:
            if not path_file_name.endswith(".json"):
                path_file_name += ".json"
            # File pathFile = new File(new File(autosDir, "paths"), pathFileName);
            autos_dir = JsonUtils.PROJECT_ROOT
            paths_folder = autos_dir / "paths"
            path_file = paths_folder / path_file_name

            # // Read entire file to String (PathPlanner approach)
            # String fileContent;
            file_content = ""
            
            # try (BufferedReader br = new BufferedReader(new FileReader(pathFile)))
            try:
                # In Python, 'with open' is the standard for 'try-with-resources'
                with open(path_file, 'r', encoding='utf-8') as f:
                    # StringBuilder sb = new StringBuilder();
                    sb = [] 
                    
                    # String line;
                    # while ((line = br.readLine()) != null)
                    for line in f:
                        # sb.append(line);
                        sb.append(line)
                    
                    # fileContent = sb.toString();
                    file_content = "".join(sb)
            except OSError as e:
                # This inner try-catch specifically handles the reading IO
                raise e

            # JSONObject json = (JSONObject) new JSONParser().parse(fileContent);
            json_data = json.loads(file_content)
            
            # return buildPathFromJson(json, loadGlobalConstraints(autosDir));
            return JsonUtils._build_path_from_json(
                json_data, 
                JsonUtils.load_global_constraints()
            )

        except (OSError, json.JSONDecodeError) as e:
            # throw new RuntimeException("Failed to load path from " + ... , e);
            error_path = f"{autos_dir}/paths/{path_file_name}"
            raise RuntimeError(f"Failed to load path from {error_path}") from e
        
    @staticmethod
    def load_path_from_json_dict(json_dict: dict, default_global_constraints: DefaultGlobalConstraints) -> Path:
        """Loads a path from a pre-parsed JSON dictionary."""
        return JsonUtils._build_path_from_json(json_dict, default_global_constraints)

    @staticmethod
    def load_path_from_json_string(path_json_str: str, default_global_constraints: DefaultGlobalConstraints) -> Path:
        """Loads a path from a JSON string."""
        try:
            json_data = json.loads(path_json_str)
            return JsonUtils._build_path_from_json(json_data, default_global_constraints)
        except json.JSONDecodeError as e:
            raise RuntimeError("Failed to parse path JSON string") from e

    @staticmethod
    def load_global_constraints() -> DefaultGlobalConstraints:
        """Loads global constraints from a config.json file."""
        autos_dir = JsonUtils.PROJECT_ROOT
            
        config_file = autos_dir / "config.json"
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                json_data = json.load(f)
            return JsonUtils._parse_default_global_constraints(json_data)
        except Exception as e:
            raise RuntimeError(f"Failed to load global constraints from {config_file}") from e
    
    @staticmethod
    def _parse_default_global_constraints(json_dict: dict) -> DefaultGlobalConstraints:
        """Parses the global constraints from a JSON object."""
        constraints_json = JsonUtils._get_nested_object(json_dict, "kinematic_constraints")
        if constraints_json is None:
            constraints_json = JsonUtils._find_best_object_containing_keys(json_dict, JsonUtils.GLOBAL_CONSTRAINT_KEY_ALIASES) or {}

        return DefaultGlobalConstraints(
            max_velocity_mps=JsonUtils._read_double_constraint(
                constraints_json, json_dict, "default_max_velocity_meters_per_sec", 
                JsonUtils.FALLBACK_GLOBAL_CONSTRAINTS.max_velocity, ["max_velocity_meters_per_sec"]),
                
            max_acceleration_ms2=JsonUtils._read_double_constraint(
                constraints_json, json_dict, "default_max_acceleration_meters_per_sec2", 
                JsonUtils.FALLBACK_GLOBAL_CONSTRAINTS.max_acceleration, ["max_acceleration_meters_per_sec2"]),
                
            max_velocity_dps=JsonUtils._read_double_constraint(
                constraints_json, json_dict, "default_max_velocity_deg_per_sec", 
                JsonUtils.FALLBACK_GLOBAL_CONSTRAINTS.max_velocity_omega, ["max_velocity_deg_per_sec"]),
                
            max_acceleration_ds2=JsonUtils._read_double_constraint(
                constraints_json, json_dict, "default_max_acceleration_deg_per_sec2", 
                JsonUtils.FALLBACK_GLOBAL_CONSTRAINTS.max_acceleration_omega, ["max_acceleration_deg_per_sec2"]),
                
            end_translation_tolerance=JsonUtils._read_double_constraint(
                constraints_json, json_dict, "default_end_translation_tolerance_meters", 
                JsonUtils.FALLBACK_GLOBAL_CONSTRAINTS.end_translation_tolerance, ["end_translation_tolerance_meters"]),
                
            end_rotation_tolerance=JsonUtils._read_double_constraint(
                constraints_json, json_dict, "default_end_rotation_tolerance_deg", 
                JsonUtils.FALLBACK_GLOBAL_CONSTRAINTS.end_rotation_tolerance, ["end_rotation_tolerance_deg"]),
                
            intermediate_handoff_radius=JsonUtils._read_double_constraint(
                constraints_json, json_dict, "default_intermediate_handoff_radius_meters", 
                JsonUtils.FALLBACK_GLOBAL_CONSTRAINTS.intermediate_handoff_radius, ["intermediate_handoff_radius_meters"])
        )


    @staticmethod
    def parse_path_components(path_json: dict, default_global_constraints: Optional[DefaultGlobalConstraints] = None) -> ParsedPathComponents:
        """Parses a path JSON object into components without constructing a Path."""
        elements = JsonUtils._parse_path_elements(path_json)
        constraints = JsonUtils._parse_path_constraints(path_json)
        
        globals_data = default_global_constraints
        globals_json = path_json.get("default_global_constraints")
        
        if globals_json is not None:
            globals_data = JsonUtils._parse_default_global_constraints(globals_json)
        elif globals_data is None:
            globals_data = JsonUtils.load_global_constraints()
            
        return JsonUtils.ParsedPathComponents(elements, constraints, globals_data)


    # ---------------------------------------------------------
    # Internal Parsing Logic
    # ---------------------------------------------------------

    @staticmethod
    def _build_path_from_json(json_dict: dict, default_global_constraints: DefaultGlobalConstraints) -> Path:
        components = JsonUtils.parse_path_components(json_dict, default_global_constraints)
        return components.to_path()

    @staticmethod
    def _parse_path_elements(json_dict: dict) -> List[PathElement]:
        elements = []
        path_elements_json = json_dict.get("path_elements", [])
        
        for element_json in path_elements_json:
            if not isinstance(element_json, dict):
                continue
                
            obj_type = element_json.get("type")

            if obj_type == "translation":
                x_meters = float(element_json.get("x_meters", 0.0))
                y_meters = float(element_json.get("y_meters", 0.0))
                handoff = element_json.get("intermediate_handoff_radius_meters")
                
                elements.append(TranslationTarget(
                    Translation2d(x_meters, y_meters),
                    float(handoff) if handoff is not None else None
                ))

            elif obj_type == "rotation":
                rot_rads = float(element_json.get("rotation_radians", 0.0))
                t_ratio = float(element_json.get("t_ratio", 0.5))
                profiled = bool(element_json.get("profiled_rotation", False))
                
                elements.append(RotationTarget(
                    Rotation2d(rot_rads),
                    t_ratio,
                    profiled
                ))

            elif obj_type == "event_trigger":
                t_ratio = float(element_json.get("t_ratio", 0.5))
                lib_key = element_json.get("lib_key")
                if lib_key is None:
                    continue
                elements.append(EventTrigger(t_ratio, str(lib_key)))

            elif obj_type == "waypoint":
                t_json = element_json.get("translation_target")
                r_json = element_json.get("rotation_target")
                
                if not t_json or not r_json:
                    continue
                    
                tx_meters = float(t_json.get("x_meters", 0.0))
                ty_meters = float(t_json.get("y_meters", 0.0))
                t_handoff = t_json.get("intermediate_handoff_radius_meters")
                
                rot_rads = float(r_json.get("rotation_radians", 0.0))
                r_tratio = float(r_json.get("t_ratio", 0.5))
                r_profiled = bool(r_json.get("profiled_rotation", False))
                
                t = TranslationTarget(
                    Translation2d(tx_meters, ty_meters),
                    float(t_handoff) if t_handoff is not None else None
                )
                r = RotationTarget(Rotation2d(rot_rads), r_tratio, r_profiled)
                
                elements.append(Waypoint(t, r))

        return elements

    @staticmethod
    def _parse_path_constraints(json_dict: dict) -> PathConstraints:
        constraints = PathConstraints()
        constraints_json = JsonUtils._get_nested_object(json_dict, "constraints")
        
        if constraints_json is None:
            constraints_json = JsonUtils._find_best_object_containing_keys(json_dict, JsonUtils.PATH_CONSTRAINT_KEY_ALIASES) or {}

        def parse_rc(key: str, setter: Callable):
            arr = JsonUtils._lookup_value_by_keys(constraints_json, json_dict, [key])
            if isinstance(arr, list) and arr:
                parsed_list = []
                for item in arr:
                    if isinstance(item, dict):
                        val = JsonUtils._to_float(item.get("value"))
                        start = JsonUtils._to_float(item.get("start_t"))
                    end = JsonUtils._to_float(item.get("end_t"))
                    if val is not None and start is not None and end is not None:
                        parsed_list.append(RangedConstraint(val, start, end))
                setter(parsed_list)

        def parse_double(key: str, setter: Callable):
            val = JsonUtils._lookup_value_by_keys(constraints_json, json_dict, [key])
            f_val = JsonUtils._to_float(val)
            if f_val is not None:
                setter(f_val)

        parse_rc("max_velocity_meters_per_sec", constraints.setMaxVelocityMps)
        parse_rc("max_acceleration_meters_per_sec2", constraints.setMaxAccelerationMps2)
        parse_rc("max_velocity_deg_per_sec", constraints.setMaxVelocityDps)
        parse_rc("max_acceleration_deg_per_sec2", constraints.setMaxAccelerationDps2)
        parse_double("end_translation_tolerance_meters", constraints.setEndTranslationTolerance)
        parse_double("end_rotation_tolerance_deg", constraints.setEndRotationToleranceDeg)

        return constraints

    @staticmethod
    def _find_best_object_containing_keys(root_json: dict, key_aliases: List[List[str]]) -> Optional[dict]:
        objects: List[dict] = []
        JsonUtils._collect_json_objects(root_json, objects)
        best = None
        best_score = 0
        for candidate in objects:
            score = sum(1 for aliases in key_aliases if any(a in candidate and candidate[a] is not None for a in aliases))
            if score > best_score:
                best_score = score
                best = candidate
        return best

    @staticmethod
    def _collect_json_objects(node: Any, out: List[dict]):
        if isinstance(node, dict):
            out.append(node)
            for child in node.values():
                JsonUtils._collect_json_objects(child, out)
        elif isinstance(node, list):
            for child in node:
                JsonUtils._collect_json_objects(child, out)

    @staticmethod
    def _get_nested_object(json_dict: dict, key: str) -> Optional[dict]:
        val = json_dict.get(key)
        return val if isinstance(val, dict) else None

    @staticmethod
    def _read_double_constraint(preferred_container: dict, root_json: dict, primary_key: str, fallback_value: float, aliases: List[str]) -> float:
        keys = [primary_key] + aliases
        raw = JsonUtils._lookup_value_by_keys(preferred_container, root_json, keys)
        if raw is None: return fallback_value
        parsed = JsonUtils._to_float(raw)
        return parsed if parsed is not None else fallback_value

    @staticmethod
    def _to_float(value: Any) -> Optional[float]:
        try: return float(value)
        except (ValueError, TypeError): return None

    @staticmethod
    def _lookup_value_by_keys(preferred_container: dict, root_json: dict, keys: List[str]) -> Any:
        for key in keys:
            if preferred_container and key in preferred_container: return preferred_container[key]
        for key in keys:
            if root_json and key in root_json: return root_json[key]
        return None