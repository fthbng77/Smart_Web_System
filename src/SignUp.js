import React, { useState } from 'react';
import axios from 'axios';
import { Link, useNavigate } from 'react-router-dom';
import { useAuth } from './Root';

function SignUp() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');

  const { setUser } = useAuth();
  const navigate = useNavigate();

  const handleSubmit = async (e) => {
    e.preventDefault();
    try {
      const response = await axios.post('http://localhost:3000/auth/signup', { email, password });
      console.log(response.data);
      
      if (response.data.user) {
        setUser(response.data.user);
        navigate('/login');
      } else {
        setError('Registration failed, please try again.');
      }
    } catch (error) {
      console.error(error);
      // Hata mesajını daha spesifik bir şekilde ayarlayın
      setError(error.response?.data?.message || 'An unexpected error occurred.');
    }
  };

  return (

    <div className="signup-container">
    <div className="logo-container">
      <img src="/gokmen512.png" alt="Logo" className="logo" />
    </div>
    <div className="form-container">
      <form onSubmit={handleSubmit} className="signup-form">
        <label>Email:
          <input type="email" value={email} onChange={(e) => setEmail(e.target.value)} />
        </label>
        <label>Password:
          <input type="password" value={password} onChange={(e) => setPassword(e.target.value)} />
        </label>
        <button type="submit">Sign Up</button>
        {/* Hata mesajını string olarak render edin */}
        {error && <p className="error">{error}</p>}
      </form>
      <div className="signup-link">
        <p>Already have an account? <Link to="/login">Log In</Link></p>
      </div>
    </div>
    </div>
  );
}

export default SignUp;
