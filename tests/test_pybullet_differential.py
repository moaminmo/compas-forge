"""Optional independent reference: identical unit boxes and world-space poses.

Bullet uses primitive boxes; Forge uses triangulated box surfaces. Compare only
separated distances / crossing boxes, never signed penetration or containment.
"""

import random
import pytest
import compas_forge as forge
from test_clearance import box, pose

bullet = pytest.importorskip('pybullet')


def test_seeded_rotated_multiscale_reference_corpus():
    from reference_corpus import run_corpus
    report = run_corpus()
    assert report['cases'] == 120
    assert report['mismatches'] == 0
    assert report['statuses'] == {'violation': 60, 'clear': 60}
    assert report['maximum_scale_normalized_witness_error'] < 1e-6


def test_pybullet_reference_agrees_on_linear_box_clearance():
    from uuid import uuid4
    mesh_id = uuid4().hex
    forge.register_mesh_to_cache(mesh_id, box())
    client = bullet.connect(bullet.DIRECT)
    try:
        shape = bullet.createCollisionShape(bullet.GEOM_BOX, halfExtents=[.5,.5,.5], physicsClientId=client)
        a = bullet.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,physicsClientId=client)
        b = bullet.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,physicsClientId=client)
        rng = random.Random(221)
        for _ in range(80):
            gap = rng.uniform(.02,.4)
            bullet.resetBasePositionAndOrientation(a,[0,1+gap,0],[0,0,0,1],physicsClientId=client)
            contacts = bullet.getClosestPoints(a,b,2,physicsClientId=client)
            reference = min(c[8] for c in contacts)
            assert reference == pytest.approx(gap,abs=1e-7)
            query = (mesh_id,pose(-3,1+gap),pose(3,1+gap),mesh_id,pose(),pose())
            result = forge.verify_clearance_cached_batch_poses([query],.2,parallel=False)[0]
            assert result['status'] == ('violation' if reference < .2 else 'clear')
            if result['status'] == 'violation':
                assert result['witness_distance'] == pytest.approx(reference,abs=1e-7)
    finally:
        bullet.disconnect(client)
        forge.unregister_mesh(mesh_id)
