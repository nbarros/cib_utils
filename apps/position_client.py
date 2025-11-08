#!/usr/bin/env python3
"""
Position Client using ZMQ
Sends position data (3 signed numbers) to a server
"""

import zmq
import json
import struct
from typing import Tuple, Optional
import random

class PositionClient:
    """
    A client class that sends position data to a ZMQ server.
    Position is defined by three signed integers (x, y, z).
    """
    
    def __init__(self, server_address: str = "tcp://localhost:5555", timeout: int = 5000):
        """
        Initialize the Position Client.
        
        Args:
            server_address: ZMQ server address (default: tcp://localhost:5555)
            timeout: Socket timeout in milliseconds (default: 5000)
        """
        self.server_address = server_address
        self.timeout = timeout
        self.context = None
        self.socket = None
        
    def connect(self):
        """Establish connection to the ZMQ server."""
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout)
        self.socket.connect(self.server_address)
        print(f"Connected to server at {self.server_address}")
        
    def disconnect(self):
        """Close the connection to the ZMQ server."""
        if self.socket:
            self.socket.close()
            self.socket = None
        if self.context:
            self.context.term()
            self.context = None
        print("Disconnected from server")
        
    def send_position_json(self, x: int, y: int, z: int) -> Optional[dict]:
        """
        Send position as JSON format.
        
        Args:
            x: X coordinate (signed integer)
            y: Y coordinate (signed integer)
            z: Z coordinate (signed integer)
            
        Returns:
            Response from server as dict, or None if error
        """
        if not self.socket:
            raise RuntimeError("Not connected. Call connect() first.")
            
        position_data = {
            "RNN800": x,
            "RNN600": y,
            "LSTAGE": z
        }
        
        try:
            # Send position data as JSON
            self.socket.send_json(position_data)
            
            # Wait for reply
            response = self.socket.recv_json()
            return response
            
        except zmq.Again:
            print("Timeout waiting for server response")
            return None
        except Exception as e:
            print(f"Error sending position: {e}")
            return None
            
    def send_position_binary(self, x: int, y: int, z: int) -> Optional[bytes]:
        """
        Send position as binary format (packed as three signed 32-bit integers).
        
        Args:
            x: X coordinate (signed integer)
            y: Y coordinate (signed integer)
            z: Z coordinate (signed integer)
            
        Returns:
            Response from server as bytes, or None if error
        """
        if not self.socket:
            raise RuntimeError("Not connected. Call connect() first.")
            
        try:
            # Pack three signed integers (little-endian)
            position_bytes = struct.pack('<iii', x, y, z)
            
            # Send position data
            self.socket.send(position_bytes)
            
            # Wait for reply
            response = self.socket.recv()
            return response
            
        except zmq.Again:
            print("Timeout waiting for server response")
            return None
        except struct.error as e:
            print(f"Error packing position data: {e}")
            return None
        except Exception as e:
            print(f"Error sending position: {e}")
            return None
            
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False


def main():
    """Example usage of PositionClient."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Send position to ZMQ server')
    parser.add_argument('--server', type=str, default='tcp://np04-iols-cib-02.cern.ch:5555',
                        help='Server address (default: tcp://np04-iols-cib-02.cern.ch:5555)')
    parser.add_argument('--x', type=int, required=True, help='X coordinate')
    parser.add_argument('--y', type=int, required=True, help='Y coordinate')
    parser.add_argument('--z', type=int, required=True, help='Z coordinate')
    parser.add_argument('--binary', action='store_true', 
                        help='Use binary format instead of JSON')
    
    args = parser.parse_args()
    
    # Use context manager for automatic connection/disconnection
    with PositionClient(server_address=args.server) as client:
        if args.binary:
            response = client.send_position_binary(args.x, args.y, args.z)
            print(f"Sent position ({args.x}, {args.y}, {args.z}) in binary format")
        else:
            response = client.send_position_json(args.x, args.y, args.z)
            print(f"Sent position ({args.x}, {args.y}, {args.z}) in JSON format")
            
        if response:
            print(f"Server response: {response}")
        else:
            print("No response received from server")

def self_test():
    """Self-test function for PositionClient."""
    with PositionClient() as client:
        # Generate random test positions
        while True:
          x, y, z = 0, random.randint(-1000000, 1000000), random.randint(-65536, 65535)
          original_position = (x, y, z)
          print("Testing JSON format...")
          response_json = client.send_position_json(x, y, z)
          print(f"Response (JSON): {response_json}")
          ret_pos = (response_json.get("RNN800"), response_json.get("RNN600"), response_json.get("LSTAGE"))
          if ret_pos != original_position:
              print(f"ERROR: Mismatched position! Sent: {original_position}, Received: {ret_pos}")
              return
          
          print("Testing binary format...")
          response_bin = client.send_position_binary(x, y, z)
          print(f"Response (Binary): {response_bin}")
          ret_pos = (response_json.get("RNN800"), response_json.get("RNN600"), response_json.get("LSTAGE"))
          if ret_pos != original_position:
              print(f"ERROR: Mismatched position! Sent: {original_position}, Received: {ret_pos}")
              return

          # sleep for 1 second between tests
          import time
          time.sleep(0.01)


if __name__ == "__main__":
    # main()
    self_test()
