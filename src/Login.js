import React, { useState } from 'react';
import axios from 'axios';
import { Link, useNavigate } from 'react-router-dom';
import { useAuth } from './Root';

function Login() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  
  const { setUser } = useAuth();
  const navigate = useNavigate(); 
  
  const handleSubmit = async (e) => {
    e.preventDefault();
    try {
      const response = await axios.post('/login', { email, password });
      console.log(response.data);
      
      setUser(response.data.user);
      navigate('/app');
      
    } catch (error) {
      console.error('Giriş başarısız:', error.response || error.message);
    }
  };

  return (
    <div className="login-container">
      <div className="logo-container">
        <img src="/gokmen512.png" alt="Logo" className="logo" />
      </div>
      <div className="form-container">
        <form onSubmit={handleSubmit} className="login-form">
          <label>Email:
            <input type="email" value={email} onChange={(e) => setEmail(e.target.value)} />
          </label>
          <label>Password:
            <input type="password" value={password} onChange={(e) => setPassword(e.target.value)} />
          </label>
          <button type="submit">Login</button>
        </form>
        <div className="signup-link">
          <p>Dont have an account? <Link to="/signup">Sign Up</Link></p>
        </div>
      </div>
    </div>
  );
}

export default Login;
