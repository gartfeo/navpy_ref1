"""The extended in-process ASGI call: query, headers, JSON, errors, deadline, cancel, id."""

from __future__ import annotations

import asyncio
from typing import Optional

from fastapi import FastAPI, HTTPException, Request

from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_asyncio import BatonEventLoop
from navpy.sim.lockstep.lockstep_asgi import EXTENSION, AsgiRequest, asgi_request

TICK_US = 20_000


def _app() -> FastAPI:
    app = FastAPI()

    # The signature of routes/vehicles.py's connect: query parameters only.
    @app.post("/api/vehicles/connect")
    async def connect(device: str, sys_id: int, name: Optional[str] = None):
        if sys_id == 99:
            raise HTTPException(status_code=500, detail=f"no vehicle {sys_id} on {device}")
        return {"status": "connected", "sys_id": sys_id, "device": device, "name": name}

    @app.put("/echo/{item}")
    async def echo(item: str, body: dict, request: Request):
        return {"item": item, "body": body, "type": request.headers["content-type"],
                "client": request.headers.get("x-client"),
                "id": request.scope["extensions"][EXTENSION]["request_id"]}

    @app.get("/slow")
    async def slow():
        await asyncio.sleep(1.0)
        return {"late": True}

    @app.get("/crash")
    async def crash():
        raise RuntimeError("boom")

    return app


def _run(requests: dict[str, AsgiRequest], *, cancel_at: int | None = None, ticks: int = 60):
    runtime, box, results = BatonRuntime(), {}, {}
    app = _app()

    def loop_thread() -> None:
        loop = BatonEventLoop(runtime)

        async def one(name: str, request: AsgiRequest) -> None:
            try:
                response = await asgi_request(app, request)
                results[name] = (response.status, response.body, runtime.now_us)
            except TimeoutError:
                results[name] = ("deadline", runtime.now_us)
            except asyncio.CancelledError:
                results[name] = ("cancelled", runtime.now_us)

        async def main() -> None:
            box["tasks"] = {name: asyncio.ensure_future(one(name, r)) for name, r in requests.items()}
            await asyncio.gather(*box["tasks"].values())

        box["loop"] = loop
        try:
            loop.run_until_complete(main())
        finally:
            loop.close()

    runtime.spawn(loop_thread, name="backend-loop")
    runtime.run_until_quiescent()
    for k in range(1, ticks):
        deliver = None
        if k == cancel_at:
            deliver = lambda: box["loop"].deliver(box["tasks"]["slow"].cancel)  # noqa: E731
        runtime.tick(k * TICK_US, deliver)
        if len(results) == len(requests):
            break
    return results


def test_query_strings_split_into_route_parameters():
    results = _run({
        "ok": AsgiRequest("POST", "/api/vehicles/connect?device=lockstep%3Aserial1&sys_id=1"),
        "named": AsgiRequest("POST", "/api/vehicles/connect?device=lockstep:serial1&sys_id=2&name=a%20b"),
    })
    assert results["ok"][:2] == (200, b'{"status":"connected","sys_id":1,"device":"lockstep:serial1","name":null}')
    assert results["named"][1].endswith(b'"name":"a b"}')


def test_json_bodies_headers_and_the_request_id_reach_the_handler():
    results = _run({"echo": AsgiRequest(
        "PUT", "/echo/x%20y", headers=(("X-Client", "operator"),), json={"params": {"A": 1.5}},
        request_id=(0, 7),
    )})
    assert results["echo"][:2] == (200, (
        b'{"item":"x y","body":{"params":{"A":1.5}},"type":"application/json",'
        b'"client":"operator","id":[0,7]}'
    ))


def test_error_statuses_and_bodies_pass_through():
    results = _run({
        "http_error": AsgiRequest("POST", "/api/vehicles/connect?device=lockstep:serial1&sys_id=99"),
        "missing_query": AsgiRequest("POST", "/api/vehicles/connect?device=lockstep:serial1"),
        "crash": AsgiRequest("GET", "/crash"),
        "unknown": AsgiRequest("GET", "/nope"),
    })
    assert results["http_error"][:2] == (500, b'{"detail":"no vehicle 99 on lockstep:serial1"}')
    assert results["missing_query"][0] == 422 and b"sys_id" in results["missing_query"][1]
    assert results["crash"][:2] == (500, b"Internal Server Error")
    assert results["unknown"][:2] == (404, b'{"detail":"Not Found"}')


def test_a_sim_time_deadline_ends_a_request_on_the_tick_grid():
    results = _run({"slow": AsgiRequest("GET", "/slow", deadline_s=0.25)})
    assert results["slow"] == ("deadline", 260_000)  # the first tick at or after 0.25 s


def test_a_cancel_ends_a_running_request_at_its_tick_and_another_stands():
    results = _run({
        "slow": AsgiRequest("GET", "/slow"),
        "ok": AsgiRequest("POST", "/api/vehicles/connect?device=lockstep:serial1&sys_id=1"),
    }, cancel_at=3)
    assert results["slow"] == ("cancelled", 60_000)
    assert results["ok"][0] == 200
