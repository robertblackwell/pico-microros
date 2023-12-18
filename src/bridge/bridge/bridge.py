import rclpy
import time
import threading
import queue
import asyncio
import collections
import serial
from typing import Any, Protocol, Union, Tuple
from rclpy.node import Node, GuardCondition

from std_msgs.msg import String, Int32

class Queue(collections.deque):
    def __init__():
        super().__init__()
    def is_empty(self):
        return len(self) == 0
    
    def get(self):
        try:
            r = self.popleft()
            return
        except:
            return None
    
    def put(self, item):
        pass

class Message(Protocol):
    def serialize(self) -> bytes:
        pass

class LineParser:
    def __init__(self):
        self.frame_buffer = b''
        self.remainder = b''
    
    def reset(self):
        self.frame_buffer = b''

    def parse(self, data: bytes) -> Union[bytes, None]:
        """Append new data to the frame that is being built.
        Return value:
            frame = f.parse(data)

            frame is either None or a byte stream representing a completed frame.
            None means that a frame is not yet complete and more data is required to be parsed

            How too use it:

                The first time f.parse() is called pass it the new data to be parsed
                thereafter call f.parse like:

                frame = f.parse(new_data)
                if frame is not None:
                    do something with the frame
        """
        i = 0
        data_len = len(data)
        while True:
            
            if data[i: i+1] == b'\n':
                if(i+1 == data_len):
                    frame = self.frame_buffer
                    self.frame_buffer = b''
                    return frame if(self.frame_buffer != b'') else None
                else:
                    self.remainder = data[i+1:]
                    frame = self.frame_buffer
                    self.frame_buffer = b''
                    return frame if(self.frame_buffer != b'') else None
            elif data[i:i+1] == b"\r":
                if(i+1 == data_len):
                    frame = self.frame_buffer
                    self.frame_buffer = b''
                    return frame
            else:
                if(i+1 == data_len):
                    self.frame_buffer = self.frame_buffer + data[i: i+1]
                    frame = self.frame_buffer
                    self.frame_buffer = b''
                    return frame
                else:
                    self.frame_buffer = self.frame_buffer + data[i: i+1]
            i = i + 1


        return (None, None)

class Transport:
    def __init__(self, guard_condition: GuardCondition ):
        self.guard_condition = guard_condition
        self.send_queue: queue.SimpleQueue = queue.SimpleQueue()
        self.recv_queue: queue.SimpleQueue = queue.SimpleQueue()        
        self.communication_thread = threading.Thread(target=self.thread_function)
        self.communication_thread.start()
        self.event_loop = asyncio.new_event_loop()
        self.write_buffer: Union[None, bytes] = None
        self.recv_buffer: Union[None, bytes] = None
        self.write_required = False
        self.serial = serial.Serial
        self.frame_parser = LineParser()
        pass

    def send(self, msg: Message):
        """Queue a message for output.
        This function is intended to be called from a thread other than the transport thread
        """
        self.event_loop.call_soon_threadsafe(self._send(msg))
        pass

    def _send(self, msg: Message):
        """This function is private to the Transport class and should only
         be run on the transport thread """
        self.send_queue.put(msg)
        self._writer()

    def recv(self) -> Any:
        """This function retreives an input message if there is one.
        return value of None indicates no input
        
        Typically called from a thread other than the transport thread"""
        r = self.recv_queue.get(block=False, timeout=0)
        return r

    def _message_complete(self, msg: Message):
        """Private to the Transport class. Called when an incoming message is complete.
        Adds the message to the recv queue and triggers the guard condition so that 
        a callback will run on the ROS2 executor thread(s)"""
        self.recv_queue.put(msg)
        self.guard_condition.trigger()

    def _reader(self):
        buffer = self.serial.read()
        frame, remainder = self.frame_parser.parse(self.parser_remainder + buffer)
        self.parser_remainder = remainder
        if frame is not None:
            msg = Message.factory(frame)
            self._message_complete(msg)

    def _writer(self):
        if not self.write_required:
            return
        if self.write_buffer is None:
            # start a new write
            try:
                self.write_buffer = self.send_queue.get(False, 0)
            except:
                self.write_buffer = None
            pass
            if self.write_buffer is None:
                # nothing to do
                return
        if self.serial.out_waiting == 0:
            return
        #
        # see if can progress the write that is in progress
        #
        n = len(self.write_buffer)
        r = self.serial.write(self.write_buffer)
        if n == r:
            self.write_buffer = None
        else:
            self.write_buffer = self.write_buffer[r:]

    def thread_function(self):
        self.new_event_loop.add_reader(self._reader)
        self.event_loop.run_forever()
        pass

class Bridge(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.count = 0
        self.publisher_ = self.create_publisher(String, 'pico_subscriber_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.fake_timer = self.create_timer(0, self.timer_trigger_callback)
        self.fake_timer.cancel()
        self.guard = self.create_guard_condition(self.gc_callback)
        self.i = 0
        self.main_thread_ident = threading.current_thread().ident
        self.t = threading.Thread(target=self.thread_function)
        self.t.start()

    def thread_function(self):
        while True:
            time.sleep(5)
            # self.fake_timer.reset()
            self.guard.trigger()

    def gc_callback(self):
        id = threading.current_thread().ident
        id2 = self.t.ident
        self.get_logger().info(f"gc__callback current thread:{id}  background:{id2}  main:{self.main_thread_ident}")
        if id != self.main_thread_ident:
            self.get_logger().info("gc_callback called on the background thread - this is wrong")            

    def timer_trigger_callback(self):
        self.get_logger().info('timer_trigger_callback')
        self.fake_timer.cancel()

    def timer_callback(self):
        msg = String()
        msg.data =  f"Here we are from python publisher {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    bridge = Bridge()

    rclpy.spin(bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()