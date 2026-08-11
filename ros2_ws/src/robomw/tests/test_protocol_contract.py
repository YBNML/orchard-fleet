import robomw.link.protocol as P


def test_new_commands_registered():
    for cmd, role in ((P.CMD_SELF_TEST, P.ROLE_OPERATOR), (P.CMD_RELOCALIZE, P.ROLE_ADMIN),
                      (P.CMD_BLACKBOX_DUMP, P.ROLE_OPERATOR), (P.CMD_WORK_STOP, P.ROLE_OPERATOR)):
        ok, _ = P.authorize(role, cmd)
        assert ok, cmd
    ok, _ = P.authorize(P.ROLE_OBSERVER, P.CMD_SELF_TEST)
    assert not ok


def test_cmd_result_shape():
    r = P.make_cmd_result("c1", "mission_start", "completed",
                          data={k: 0 for k in P.MISSION_REPORT_KEYS})
    assert r["kind"] == "cmd_result" and r["status"] == "completed" and r["code"] == "OK"
    assert set(P.MISSION_REPORT_KEYS) <= set(r["data"])


def test_cmd_result_rejects_bad_status():
    import pytest
    with pytest.raises(ValueError):
        P.make_cmd_result("c1", "ping", "definitely-not-a-status")


def test_validate_work():
    assert P.validate_work({"type": "scout"})[0]
    assert P.validate_work({"type": "spray", "params": {"speed_scale": 0.5}})[0]
    assert not P.validate_work({"type": "teleport"})[0]
    assert not P.validate_work({"type": "mow", "params": {"speed_scale": 3.0}})[0]


def test_topic_site_generalized():
    assert P.topic("orchard", "scout01", "cmd") == "orchard/scout01/cmd"
    assert P.topic("factory7", "biped01", "event") == "factory7/biped01/event"
