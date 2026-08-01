import pytest
from fleet_server.db import Base, make_engine, make_session_factory


@pytest.fixture()
def db():
    engine = make_engine("sqlite://")          # in-memory
    Base.metadata.create_all(engine)
    factory = make_session_factory(engine)
    with factory() as session:
        yield session
