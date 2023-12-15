import React, { useState, useCallback, useMemo } from 'react';
import ROSLIB from 'roslib';

function JoystickComponent() {
  const [position, setPosition] = useState({ x: 0, y: 0 });

  const ros = useMemo(() => new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  }), []);

  const topic = useMemo(() => new ROSLIB.Topic({
    ros: ros,
    name: '/mavros/setpoint_velocity/cmd_vel',
    messageType: 'geometry_msgs/TwistStamped'
  }), [ros]);

  const handleMouseMove = useCallback((event) => {
    const rect = event.currentTarget.getBoundingClientRect();
    const x = event.clientX - rect.left - rect.width / 2;
    const y = event.clientY - rect.top - rect.height / 2;

    setPosition({ x, y });

    const currentTime = new Date();
    const command = new ROSLIB.Message({
      header: {
        seq: 0,
        stamp: {
          secs: Math.floor(currentTime.getTime() / 1000),
          nsecs: (currentTime.getTime() % 1000) * 1e6
        },
        frame_id: ""
      },
      twist: {
        linear: { x: position.x, y: position.y, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      }
    });

    topic.publish(command);
  }, [position.x, position.y, topic]);

  const handleMouseLeave = useCallback(() => {
    setPosition({ x: 0, y: 0 });

    const currentTime = new Date();
    const stopCommand = new ROSLIB.Message({
      header: {
        seq: 0,
        stamp: {
          secs: Math.floor(currentTime.getTime() / 1000),
          nsecs: (currentTime.getTime() % 1000) * 1e6
        },
        frame_id: ""
      },
      twist: {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      }
    });

    topic.publish(stopCommand);
  }, [topic]);

  return (
    <div
      style={{
        width: '100px',
        height: '100px',
        backgroundColor: '#eee',
        position: 'relative',
        borderRadius: '50%',
      }}
      onMouseMove={handleMouseMove}
      onMouseLeave={handleMouseLeave}
    >
      <div
        style={{
          width: '20px',
          height: '20px',
          backgroundColor: 'red',
          borderRadius: '50%',
          position: 'absolute',
          left: `calc(50% + ${position.x}px)`,
          top: `calc(50% + ${position.y}px)`,
          transform: 'translate(-50%, -50%)',
        }}
      ></div>
    </div>
  );
}

export default JoystickComponent;