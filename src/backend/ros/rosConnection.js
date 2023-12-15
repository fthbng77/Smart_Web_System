// ros/rosConnection.js
const roslib = require('roslib');

function setupRosConnection(io) {
  io.on('connection', (socket) => {
    console.log('Client connected');

    const ros = new roslib.Ros({ url: 'ws://localhost:9090' });

    ros.on('connection', () => {
      console.log('Connected to ROS websocket server');

      const topic = new roslib.Topic({
        ros,
        name: '/webcam/image_raw/compressed',
        messageType: 'sensor_msgs/CompressedImage',
      });

      topic.subscribe((message) => {
        console.log('Received message from ROS topic', message);
        socket.emit('imageData', message);
      });
    });

    ros.on('error', (error) => {
      console.log('Error connecting to ROS websocket server: ', error);
      socket.emit('rosError', error);
    });

    ros.on('close', () => {
      console.log('ROS websocket server connection closed');
      socket.emit('rosClosed');
    });

    socket.on('disconnect', () => {
      console.log('Client disconnected');
      topic.unsubscribe();
    });
  });
}

module.exports = setupRosConnection;
