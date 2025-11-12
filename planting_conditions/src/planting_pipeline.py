#!/usr/bin/env python3
import rclpy, asyncio
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import asyncio

class PlantingPipeline(Node):
    def __init__(self):
        super().__init__('planting_pipeline')

        # Topics 
        self.goal_received_ = False
        self.ready_to_plant_ = False
        self.rejected_ = False
        self.plant_done_ = False
        self.cover_done_ = False
        self.irrigate_done_ = False
        self.running_ = False

        self.create_subscription(Bool, '/mission/goal_received', self._on_goal_received, 10)
        self.create_subscription(Bool, '/soil_prep/ready_to_plant', self._on_ready, 10)
        self.create_subscription(Bool, '/soil_prep/rejected', self._on_reject, 10)
        self.create_subscription(Bool, '/plant/done', self._on_plant_done, 10)
        self.create_subscription(Bool, '/cover/done', self._on_cover_done, 10)
        self.create_subscription(Bool, '/irrigate/done', self._on_irrigate_done, 10)
        

        # Service clients 
        self.soil_prep_cli = self.create_client(Trigger, '/soil_prep/start')
        self.plant_cli     = self.create_client(Trigger, '/plant/start')
        self.cover_cli     = self.create_client(Trigger, '/cover/start')
        self.irrigate_cli  = self.create_client(Trigger, '/irrigate/start')
        self.done_pub = self.create_publisher(Bool, '/mission/planting_done', 10)
        self.status_pub = self.create_publisher(String, '/mission/fsm_status', 10)
        self._phase = None  

        # Kick off the async pipeline after startup
        self.timer_ = self.create_timer(0.2, self._tick)
        self._started = False

    # Status publishing
    def _set_phase(self, phase: str):
        # Publish status only when phase changes.
        if self._phase != phase:
            self._phase = phase
            msg = String()
            msg.data = f'FSM: {phase}'
            self.status_pub.publish(msg)
            self.get_logger().info(f'STATUS is {msg.data}')

    def _clear_phase(self):
        self._phase = None  

    #Topic callbacks
    def _on_goal_received(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Pipeline: mission goal received.')
            self.goal_received_ = True
    def _on_ready(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Soil prep: ready_to_plant = true')
            self.ready_to_plant_ = True

    def _on_reject(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('Soil prep: REJECT = true')
            self.rejected_ = True

    def _on_plant_done(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Planting: done = true')
            self.plant_done_ = True

    def _on_cover_done(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Cover: done = true')
            self.cover_done_ = True

    def _on_irrigate_done(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Irrigation: done = true')
            self.irrigate_done_ = True

    # ---- Helpers
    async def _wait_for(self, condition_fn, desc, timeout=None):
        self.get_logger().info(f'Waiting for: {desc} ...')
        t0 = self.get_clock().now()
        while rclpy.ok() and not condition_fn():
            await asyncio.sleep(0.1)
            if timeout is not None:
                if (self.get_clock().now() - t0).nanoseconds * 1e-9 > timeout:
                    raise TimeoutError(f'Timeout waiting for: {desc}')
        self.get_logger().info(f'Got: {desc}')

    async def _call_trigger(self, cli, name, wait_svc_timeout=5.0):
        self.get_logger().info(f'Calling {name} ...')
        # Wait for service
        if not await self._wait_for_service(cli, name, wait_svc_timeout):
            raise RuntimeError(f'Service {name} not available')
        req = Trigger.Request()
        fut = cli.call_async(req)
        while rclpy.ok() and not fut.done():
            await asyncio.sleep(0.05)
        res = fut.result()
        if not res or not res.success:
            raise RuntimeError(f'{name} failed: {res.message if res else "no response"}')
        self.get_logger().info(f'{name} → {res.message}')

    async def _wait_for_service(self, cli, name, timeout):
        t0 = self.get_clock().now()
        while rclpy.ok() and not cli.wait_for_service(timeout_sec=0.2):
            await asyncio.sleep(0.1)
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 > timeout:
                self.get_logger().error(f'Timeout waiting for service {name}')
                return False
        return True

    def _tick(self):
        # if we got a goal AND we’re not already running 
        if self.goal_received_ and not self.running_:
            self.running_ = True
            asyncio.ensure_future(self._run_pipeline())

    async def _run_pipeline(self):
        try:
            # wait for mission to confirm goal was actually received
            await self._wait_for(lambda: self.goal_received_, '/mission/goal_received == true')

            # 1) Start soil prep
            self._set_phase('CHECKING CONDITIONS')
            await self._call_trigger(self.soil_prep_cli, '/soil_prep/start')

            # 2) Wait for either reject or ready_to_plant
            self.get_logger().info('Waiting for soil decision...')
            while rclpy.ok():
                if self.rejected_:
                    self.get_logger().warn('Pipeline abort: soil rejected.')
                    self._set_phase('BAD CONDITIONS')
                    return
                if self.ready_to_plant_:
                    break
                await asyncio.sleep(0.1)
            # self._set_phase('RAKING')
            self.get_logger().info('Soil accepted → continue.')

            # 3) Start planting and wait for done
            self._set_phase('SEEDING')
            await self._call_trigger(self.plant_cli, '/plant/start')
            await self._wait_for(lambda: self.plant_done_, 'plant done')

            # 4) Start cover and wait for done
            self._set_phase('COVERING')
            await self._call_trigger(self.cover_cli, '/cover/start')
            await self._wait_for(lambda: self.cover_done_, 'cover done')

            # 5) Start irrigation and wait for done
            self._set_phase('IRRIGATING')
            await self._call_trigger(self.irrigate_cli, '/irrigate/start')
            await self._wait_for(lambda: self.irrigate_done_, 'irrigation done')

            self.get_logger().info('Pipeline finished successfully.')

            # 6) Once done with all tasks, send new message to tell FSM to continue
            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)
        except Exception as e:
            self.get_logger().error(f'Pipeline error: {e}')
        finally:
            self._reset_flags()

    def _reset_flags(self):
        self.goal_received_ = False
        self.ready_to_plant_ = False
        self.rejected_ = False
        self.plant_done_ = False
        self.cover_done_ = False
        self.irrigate_done_ = False
        self.running_ = False
        self._clear_phase()
        self.get_logger().info('Pipeline: reset, waiting for next /mission/goal_received')


def main():
    rclpy.init()
    node = PlantingPipeline()

    # Spin rclpy in an asyncio loop
    loop = asyncio.get_event_loop()
    try:
        executor = rclpy.executors.SingleThreadedExecutor()
        async def spin():
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                await asyncio.sleep(0.01)
        loop.create_task(spin())
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()