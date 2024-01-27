const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const mongoose = require('mongoose');
const path = require('path');
const cors = require('cors');
const authRoutes = require('./routes/auth');
const setupRosConnection = require('./ros/rosConnection');  

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

// MongoDB'ye Bağlanma
mongoose.connect('mongodb://localhost/gokmendatabase', {
  useNewUrlParser: true,
  useUnifiedTopology: true,
});

// Middleware Tanımlama
app.use(express.json());  
app.use(cors());
app.use('/auth', authRoutes);

setupRosConnection(io);

// Statik Dosyaları Sunma
app.use(express.static('/home/fatih/catkin_ws/src/iq_gnc/scripts/Gokmen/gokmen-app/build'));

app.get('*', (req, res) => {
  res.sendFile(path.resolve('/home/fatih/catkin_ws/src/iq_gnc/scripts/Gokmen/gokmen-app/build', 'index.html'));
});

server.listen(3000, () => {
  console.log('Server is listening on port 3000');
});
