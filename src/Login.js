import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { Link, useNavigate } from 'react-router-dom';
import { useAuth } from './Root';

function Login() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');

  const auth = useAuth();
  const navigate = useNavigate();

  useEffect(() => {
    if (auth.isLoggedIn) {
      navigate('/app');
    }
  }, [auth.isLoggedIn, navigate]);

  const handleSubmit = async (e) => {
    e.preventDefault();

    try {
      const response = await axios.post('http://localhost:3000/auth/login', { email, password }, {
        headers: {
          'Content-Type': 'application/json'
        },
        withCredentials: true // Important for CORS and cookies if you use them
      });

      if (response.data.token) {
        localStorage.setItem('authToken', response.data.token);
        auth.setUser(response.data.user); // Update user state
        navigate('/app');
      } else {
        setError('Login failed. Please check your credentials.');
      }
    } catch (err) {
      setError(err.response?.data?.message || 'An unexpected error occurred during login.');
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
            <input 
              type="email" 
              value={email} 
              onChange={(e) => setEmail(e.target.value)} 
              required 
            />
          </label>
          <label>Password:
            <input 
              type="password" 
              value={password} 
              onChange={(e) => setPassword(e.target.value)} 
              required 
            />
          </label>
          <button type="submit">Login</button>
          {error && <div className="error-message">{error}</div>}
        </form>
        <div className="signup-link">
          <p>Dont have an account? <Link to="/signup">Sign Up</Link></p>
        </div>
      </div>
    </div>
  );
}

export default Login;

