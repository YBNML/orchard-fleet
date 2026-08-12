import numpy as np
from robomw.core.blackbox import Blackbox


def test_dump_roundtrip(tmp_path):
    b = Blackbox()
    for i in range(100):
        b.feed_pose(1000.0 + i, float(i), 0.0, 0.0)
    b.feed_event({"kind": "estop", "t": 1050.0})
    out = b.dump(str(tmp_path / "bb.npz"), window_s=50)
    d = np.load(out["path"], allow_pickle=False)
    assert d["poses"].shape[1] == 4
    assert d["poses"][:, 0].min() >= 1000.0 + 100 - 50 - 1   # window 절단
    assert out["events"] == 1 and out["poses"] == len(d["poses"])


def test_window_capped_at_900():
    b = Blackbox()
    assert b.effective_window(5000) == 900
