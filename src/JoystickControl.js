import React, { useState, useEffect, useMemo } from 'react';
import ROSLIB from 'roslib';

function JoystickControl() {
  const [position, setPosition] = useState({ x: 0, y: 0, z: 0 });

  const ros = useMemo(() => new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  }), []);

  const topic = useMemo(() => new ROSLIB.Topic({
    ros: ros,
    name: '/mavros/setpoint_velocity/cmd_vel',
    messageType: 'geometry_msgs/TwistStamped'
  }), [ros]);

  const speedMultiplier = 5;

  useEffect(() => {
    const gamepadUpdate = setInterval(() => {
      const gamepads = navigator.getGamepads();
      if (gamepads[0]) {
        const x = gamepads[0].axes[0] * speedMultiplier;
        const y = gamepads[0].axes[1] * speedMultiplier;
        const z = gamepads[0].axes[2] * speedMultiplier; // Sağ joystick'in yatay hareketi

        // Z ekseni etrafında dönüş açısını al
        const angle = z;

        // Açıya göre hareket vektörünü döndür
        const rotatedX = x * Math.cos(angle) - y * Math.sin(angle);
        const rotatedY = x * Math.sin(angle) + y * Math.cos(angle);

        setPosition({ x, y, z });

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
            linear: { x: rotatedX, y: rotatedY, z: 0 },
            angular: { x: 0, y: 0, z: angle }
          }
        });

        topic.publish(command);
      }
    }, 100);

    return () => clearInterval(gamepadUpdate);
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
    >
      <div
        style={{
          width: '20px',
          height: '20px',
          backgroundColor: 'red',
          borderRadius: '50%',
          position: 'absolute',
          left: `calc(50% + ${position.x * 5}px)`,
          top: `calc(50% + ${position.y * 5}px)`,
          transform: 'translate(-50%, -50%)',
        }}
      ></div>
    </div>
  );
}

export default JoystickControl;
