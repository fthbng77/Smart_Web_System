import React, { useContext, createContext, useState } from "react";
import PropTypes from "prop-types";
import { BrowserRouter as Router, Route, Routes, Navigate } from "react-router-dom";
import App from "./App";
import Login from "./Login";
import SignUp from "./SignUp";
import RosImagePage from "./RosImagePage";

const AuthContext = createContext();

export function useAuth() {
  return useContext(AuthContext);
}

export function AuthProvider({ children }) {
  const [user, setUser] = useState(null);

  const value = {
    user,
    setUser,
    isLoggedIn: Boolean(user),
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

AuthProvider.propTypes = {
  children: PropTypes.node.isRequired,
};

function PrivateRoute({ children }) {
  const { isLoggedIn } = useAuth();

  return isLoggedIn ? (
    children
  ) : (
    <Navigate to="/login" replace />
  );
}

PrivateRoute.propTypes = {
  children: PropTypes.node.isRequired,
};

function Root() {
  return (
    <AuthProvider>
      <Router>
        <Routes>
          <Route path="/app" element={<PrivateRoute><App /></PrivateRoute>} />
          <Route path="/ros-image" element={<PrivateRoute><RosImagePage /></PrivateRoute>} />
          <Route path="/login" element={<Login />} />
          <Route path="/signup" element={<SignUp />} />
          <Route index element={<Navigate to="/app" replace />} /> {/* Varsayılan olarak /app sayfasına yönlendir */}
          <Route path="*" element={<Navigate to="/app" replace />} /> {/* Tanımsız rotalar için /app sayfasına yönlendir */}
        </Routes>
      </Router>
    </AuthProvider>
  );
}

export default Root;