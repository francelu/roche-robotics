width = 1024
hfov = 90
height = width

from typing import List, Dict, Any, Tuple, Optional
import magnum as mn
from habitat_sim.physics import ManagedRigidObject

class ObjectTemplateContainer:
    def __init__(self, obj_path: str, scale: List[float]) -> None:
        self.obj_path: str = obj_path
        self.scale: np.ndarray = np.array(scale, dtype=float)
        self.handle = self._propose_handle(
            obj_path=obj_path,
            scale=scale  # use list
        )

    def _propose_handle(self, obj_path: str, scale: List[float]) -> str:
        base, suffix = obj_path.split('.')
        for s in scale:
            base += '_' + self._smart_round(s)
        return base + '.' + suffix

    @staticmethod
    def _smart_round(val: float, n_places: int = 3) -> str:
        rounded = round(val, n_places)
        if rounded == int(val):  # if val is an int
            return str(int(val))  # return val as str
        else:
            return str(rounded).replace('.', '-')  # return rounded number as str

    def __eq__(self, other) -> bool:
        return other.obj_path == self.obj_path \
            and np.allclose(other.scale, self.scale)

    def eq(self, other_obj_path: str, other_scale: List[float]):
        return other_obj_path == self.obj_path \
            and np.allclose(other_scale, self.scale)

class RunTimeObjectManager:
    def __init__(self, sim: habitat_sim.Simulator) -> None:
        self._sim = sim
        self._obj_templates_mgr = self._sim.get_object_template_manager()
        self._rigid_obj_mgr = self._sim.get_rigid_object_manager()

        self._navmesh_settings = habitat_sim.NavMeshSettings()
        self._navmesh_settings.set_defaults()

        self._raw_tpl_ctrs: List[ObjectTemplateContainer] = []
        self._scaled_tpl_ctrs: List[ObjectTemplateContainer] = []

        self._added_target: Optional[ManagedRigidObject] = None
        self._added_walls: List[ManagedRigidObject] = []
        self._added_obstacles: List[ManagedRigidObject] = []

    def add_walls(self, walls_layout_cfg: List[Dict[str, Any]]) -> None:
        for wall_cfg in walls_layout_cfg:
            wall_obj = self._add_static_obj(
                obj_path=wall_cfg['asset'],
                position=wall_cfg['pos'],
                rotation=mn.Quaternion.rotation(
                    mn.Deg(wall_cfg['rot_amount']), wall_cfg['rot_vector']
                ),
                scale=wall_cfg['scale'],
            )

            self._added_walls.append(wall_obj)

    def add_obstacles(self, obstacles_layout_cfg: List[Dict[str, Any]]) -> None:
        for obstacle_cfg in obstacles_layout_cfg:
            obstacle_obj = self._add_static_obj(
                obj_path=obstacle_cfg['asset'],
                position=obstacle_cfg['pos'],
                rotation=mn.Quaternion.rotation(
                    mn.Deg(obstacle_cfg['rot_amount']),
                    obstacle_cfg['rot_vector']
                ),
                scale=obstacle_cfg['scale'],
            )

            self._added_obstacles.append(obstacle_obj)

    def add_target(self, target_path: str, position: List[float]) -> None:
        target_obj = self._add_static_obj(
            obj_path=target_path,
            position=position,
            rotation=mn.Quaternion.rotation(
                mn.Deg(0.0),
                [0.0, 1.0, 0.0]
            ),  # Goal in PointNav has no rotation
            # scale=[1.0, 1.0, 1.0],  # Maybe various scales?
            scale=[1.0, 1.0, 1.0],
        )

        self._added_target = target_obj

    def _add_static_obj(
        self, obj_path: str, scale: List[float],
        position: List[float], rotation: mn.Quaternion.rotation
    ) -> ManagedRigidObject:
        has_raw_tpl, raw_tpl_ctr = self._has_raw_template(
            obj_path=obj_path
        )
        if not has_raw_tpl:  # Load raw tpl
            raw_obj_tpl_id = self._obj_templates_mgr.load_configs(
                path=obj_path
            )[0]
            raw_obj_tpl = self._obj_templates_mgr.get_template_by_id(
                template_id=raw_obj_tpl_id
            )
            raw_scale = raw_obj_tpl.scale
            raw_tpl_ctr = ObjectTemplateContainer(
                obj_path=obj_path,
                scale=raw_scale
            )
            # Raw handle is from mgr
            raw_tpl_ctr.handle = self._obj_templates_mgr.get_template_handle_by_id(
                template_id=raw_obj_tpl_id
            )
            self._raw_tpl_ctrs.append(raw_tpl_ctr)
        else:
            raw_obj_tpl = self._obj_templates_mgr.get_template_by_handle(
                handle=raw_tpl_ctr.handle
            )

        has_scaled_tpl, scaled_tpl_ctr = self._has_scaled_template(
            obj_path=obj_path,
            scale=scale
        )
        if not has_scaled_tpl:
            scaled_tpl_ctr = ObjectTemplateContainer(
                obj_path=obj_path,
                scale=scale
            )

            raw_obj_tpl.scale = scale
            self._obj_templates_mgr.register_template(
                template=raw_obj_tpl,
                specified_handle=scaled_tpl_ctr.handle
            )
            self._scaled_tpl_ctrs.append(scaled_tpl_ctr)

        obj: ManagedRigidObject = \
            self._rigid_obj_mgr.add_object_by_template_handle(
                object_lib_handle=scaled_tpl_ctr.handle
            )

        obj.translation = mn.Vector3(position)
        obj.rotation = rotation
        obj.motion_type = habitat_sim.physics.MotionType.STATIC

        return obj

    def _has_raw_template(
        self, obj_path: str
    ) -> Tuple[bool, Optional[ObjectTemplateContainer]]:
        for tpl_ctr in self._raw_tpl_ctrs:
            if tpl_ctr.obj_path == obj_path:
                return True, tpl_ctr
        return False, None

    def _has_scaled_template(
        self, obj_path: str, scale: List[float]
    ):
        for tpl_ctr in self._scaled_tpl_ctrs:
            if tpl_ctr.eq(obj_path, scale):
                return True, tpl_ctr
        return False, None

    def delete_added_target(self) -> None:
        if self._added_target is None:
            return

        self._rigid_obj_mgr.remove_object_by_id(
            object_id=self._added_target.object_id
        )
        self._added_target = None

    def delete_added_walls(self) -> None:
        if len(self._added_walls) <= 0:
            return
        while len(self._added_walls) > 0:
            wall_to_rm = self._added_walls.pop()
            self._rigid_obj_mgr.remove_object_by_id(
                object_id=wall_to_rm.object_id
            )

    def delete_added_obstacles(self) -> None:
        if len(self._added_obstacles) <= 0:
            return
        while len(self._added_obstacles) > 0:
            obstacle_to_rm = self._added_obstacles.pop()
            self._rigid_obj_mgr.remove_object_by_id(
                object_id=obstacle_to_rm.object_id
            )

    def recompute_navmesh(self) -> None:
        self._sim.recompute_navmesh(
            pathfinder=self._sim.pathfinder,
            navmesh_settings=self._navmesh_settings,
            include_static_objects=True  # Take int account added objects
        )

    def refresh(self, sim: habitat_sim.Simulator) -> None:
        if sim is not self._sim:  # new sim
            self._sim = sim
            self._obj_templates_mgr = self._sim.get_object_template_manager()

            # Clear
            self._raw_tpl_ctrs: List[ObjectTemplateContainer] = []
            self._scaled_tpl_ctrs: List[ObjectTemplateContainer] = []

        if self._sim.get_rigid_object_manager() is not self._rigid_obj_mgr:  # sim reconfigured
            self._rigid_obj_mgr = self._sim.get_rigid_object_manager()

            # Clear
            self._added_target: Optional[ManagedRigidObject] = None
            self._added_walls: List[ManagedRigidObject] = []
            self._added_obstacles: List[ManagedRigidObject] = []