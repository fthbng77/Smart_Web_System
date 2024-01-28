import React, { useState, useEffect, useRef, useCallback } from "react";
import ROSLIB from "roslib";

function DroneControl() {
  const [altitude, setAltitude] = useState(5);
  const [connectionStatus, setConnectionStatus] = useState("Disconnected");
  const rosRef = useRef(new ROSLIB.Ros({
    url: "ws://localhost:9090",
  }));

  const armDrone = (arm) => {
    const request = new ROSLIB.ServiceRequest({
      value: arm,
    });

    const armService = new ROSLIB.Service({
      ros: rosRef.current,
      name: "/mavros/cmd/arming",
      serviceType: "mavros_msgs/CommandBool",
    });

    armService.callService(request, (result) => {
      console.log(result);
    });
  };

  const changeMode = (mode) => {
    const request = new ROSLIB.ServiceRequest({
      custom_mode: mode,
    });

    const setModeService = new ROSLIB.Service({
      ros: rosRef.current,
      name: "/mavros/set_mode",
      serviceType: "mavros_msgs/SetMode",
    });

    setModeService.callService(request, (result) => {
      console.log(result);
    });
  };

  const sendVelocityCommand = (linearX, linearY, linearZ) => {
    const twist = new ROSLIB.Message({
      linear: {
        x: linearX,
        y: linearY,
        z: linearZ,
      },
    });

    const velocityTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/mavros/setpoint_velocity/cmd_vel_unstamped",
      messageType: "geometry_msgs/Twist",
    });

    velocityTopic.publish(twist);
  };

  const takeoff = () => {
    const request = new ROSLIB.ServiceRequest({
      altitude: altitude,
    });

    const takeoffService = new ROSLIB.Service({
      ros: rosRef.current,
      name: "/mavros/cmd/takeoff",
      serviceType: "mavros_msgs/CommandTOL",
    });

    takeoffService.callService(request, (result) => {
      console.log(result);
    });
  };

  const handleKeyDown = useCallback((e) => {
    switch (e.key) {
      case 'a':
        sendVelocityCommand(1, 0, 0);
        break;
      case 'w':
        sendVelocityCommand(0, -1, 0);
        break;
      case 'd':
        sendVelocityCommand(-1, 0, 0);
        break;
      case 's':
        sendVelocityCommand(0, 1, 0);
        break;
      // Diğer tuşlar için daha fazla case ekleyebilirsiniz
    }
  }, []);

  const handleKeyUp = useCallback(() => {
    sendVelocityCommand(0, 0, 0);
  }, []);

  useEffect(() => {
    const currentRosRef = rosRef.current;
    
    const connectToRos = () => {
      currentRosRef.connect('ws://localhost:9090');
    };
  
    currentRosRef.on("connection", function () {
      console.log("Connected to websocket server.");
      setConnectionStatus("Connected");
    });
  
    currentRosRef.on("error", function (error) {
      console.log("Error connecting to websocket server: ", error);
      setConnectionStatus("Error");
    });
  
    currentRosRef.on("close", function () {
      console.log("Connection to websocket server closed.");
      setConnectionStatus("Disconnected");
      setTimeout(connectToRos, 3000);
    });
  
    connectToRos();

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      currentRosRef.close();
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [altitude, handleKeyDown, handleKeyUp]);

  return (
    <div style={{
      padding: '10px',
      border: '1px solid #ddd',
      borderRadius: '8px',
      flex: 1,
      display: 'flex',
      flexDirection: 'column',
      gap: '10px',
      background: '#fff',
      boxShadow: '0px 0px 10px rgba(0,0,0,0.1)'
    }}>
      <div>
        <strong>Bağlanti Durumu:</strong> {connectionStatus}
      </div>
      <div style={{ display: 'grid', gap: '10px', marginBottom: '10px' }}>
        <button onClick={() => armDrone(true)}>ARM</button>
        <button onClick={() => armDrone(false)}>DISARM</button>
        <button onClick={() => changeMode("GUIDED")}>GUIDED MODE</button>
      </div>
      <div style={{ display: 'grid', gridTemplateColumns: 'auto auto auto', gap: '10px', justifyContent: 'center', marginBottom: '10px' }}>
        <div></div>
        <button onClick={() => sendVelocityCommand(0, -1, 0)}>Forward</button>
        <div></div>
        <button onClick={() => sendVelocityCommand(1, 0, 0)}>Left</button>
        <div></div>
        <button onClick={() => sendVelocityCommand(-1, 0, 0)}>Right</button>
        <div></div>
        <button onClick={() => sendVelocityCommand(0, 1, 0)}>Backward</button>
        <div></div>
      </div>
      <div style={{ display: 'grid', gridTemplateColumns: '1fr auto', gap: '10px', alignItems: 'center' }}>
        <input
          type="number"
          value={altitude}
          onChange={(e) => setAltitude(Number(e.target.value))}
          style={{ width: '60px' }}
        />
        <button onClick={takeoff}>Takeoff</button>
      </div>
    </div>
  );
}

export default DroneControl;
