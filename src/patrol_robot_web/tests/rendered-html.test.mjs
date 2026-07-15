import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

async function render() {
  const workerUrl = new URL("../dist/server/index.js", import.meta.url);
  workerUrl.searchParams.set("test", `${process.pid}-${Date.now()}`);
  const { default: worker } = await import(workerUrl.href);

  return worker.fetch(
    new Request("http://localhost/", {
      headers: { accept: "text/html" },
    }),
    {
      ASSETS: {
        fetch: async () => new Response("Not found", { status: 404 }),
      },
    },
    {
      waitUntil() {},
      passThroughOnException() {},
    },
  );
}

test("server-renders the patrol robot control console", async () => {
  const response = await render();
  assert.equal(response.status, 200);
  assert.match(response.headers.get("content-type") ?? "", /^text\/html\b/i);

  const html = await response.text();
  assert.match(html, /<title>巡检智控 · 车辆控制台<\/title>/i);
  assert.match(html, /车辆控制台/);
  assert.match(html, /2D 地图/);
  assert.match(html, /3D 场景/);
  assert.match(html, /摄像画面/);
  assert.match(html, /人工控制/);
  assert.match(html, /前进/);
  assert.match(html, /后退/);
});

test("includes live ROS controls and both map implementations", async () => {
  const [page, map2d, map3d, bridge] = await Promise.all([
    readFile(new URL("../app/page.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/Industrial2DMap.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/Industrial3DMap.tsx", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_web_bridge/patrol_robot_web_bridge/bridge_node.py", import.meta.url), "utf8"),
  ]);

  assert.match(page, /\/api\/status/);
  assert.match(page, /\/api\/control\/manual/);
  assert.match(page, /\/api\/navigation\/waypoints/);
  assert.match(page, /\/api\/camera\/enable/);
  assert.match(page, /\/api\/camera\/stream/);
  assert.match(page, /\/api\/camera\/gimbal/);
  assert.match(page, /开启摄像头/);
  assert.match(page, /回到正前方/);
  assert.match(page, /<Industrial2DMap/);
  assert.match(page, /<Industrial3DMap/);
  assert.match(map2d, /industrial-2d-map/);
  assert.match(map2d, /巡检工程平面图/);
  assert.match(map3d, /industrial-3d-map/);
  assert.match(bridge, /\/api\/control\/manual/);
  assert.match(bridge, /\/api\/camera\/enable/);
  assert.match(bridge, /\/api\/camera\/stream/);
  assert.match(bridge, /\/api\/camera\/gimbal/);
  assert.match(bridge, /camera_stream_fps', 12\.0/);
  assert.match(bridge, /manual_command_timeout/);
});
