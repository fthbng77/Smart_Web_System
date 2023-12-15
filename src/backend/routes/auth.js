// routes/auth.js
const express = require('express');
const bcrypt = require('bcryptjs');
const User = require('../models/user');  // models klasörünün yolunu doğru belirtin
const router = express.Router();

router.post('/register', async (req, res) => {
  try {
    const { email, password } = req.body;
    const hashedPassword = await bcrypt.hash(password, 10);
    const newUser = new User({ email, password: hashedPassword });
    await newUser.save();
    res.status(200).send('User registered successfully');
  } catch (error) {
    console.error(error);
    res.status(500).send('Server Error');
  }
});

router.post('/login', async (req, res) => {
  try {
    // ... giriş işlemleri
  } catch (error) {
    console.error(error);
    res.status(500).send('Server Error');
  }
});

module.exports = router;
