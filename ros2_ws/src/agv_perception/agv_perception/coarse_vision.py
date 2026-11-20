"""Production lifecycle adapter for the pipeline used by regional debugging."""

from .work_vision import RingVision


class CoarseVision:
    def __init__(self, config, logger, raw_config=None):
        raw = raw_config or config
        if raw['material_config'] != config['material_config']:
            raise ValueError('原料与暂存物料配置必须共用同一主相机')
        self.camera = RingVision(config['ring_config'], 960, logger, raw['material_config'])
        self._active = False
        self._kind = ''

    @property
    def ready(self):
        return self.camera.ready

    @property
    def error(self):
        return self.camera.error

    def request(self, active, kind='', color=0, area='coarse', *, geometry_target=0):
        self._active, self._kind = active, kind
        if not active or not kind:
            # Invalidate in-flight results across fixed arm actions.
            self.camera.request('')
        else:
            self.camera.request(kind, geometry_target=geometry_target, largest_per_color=area == 'raw')

    def latest(self):
        return self.camera.latest() if self._active and self._kind else None

    def close(self):
        self.camera.close()
