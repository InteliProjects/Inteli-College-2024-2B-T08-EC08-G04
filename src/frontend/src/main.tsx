import React from 'react'
import ReactDOM from 'react-dom/client'
import { BrowserRouter, Routes, Route } from 'react-router-dom'
import App from './App'
import Home from './pages/Home' // Adjust the path as needed
import Conversation from "./pages/Conversation";
import Login from './pages/Login'
import Signup from './pages/Signup'
import Signup_pac from './pages/Signup_pac'
import Home_app from './pages/Home_app'
import './index.css'
import ResponseOverlay from './components/ResponseOverlay'
import Perfil from './pages/Perfil'

ReactDOM.createRoot(document.getElementById('root') as HTMLElement).render(
  <React.StrictMode>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<ResponseOverlay text='Lorem Ipsum is simply dummy text of the printing and typesetting industry. Lorem Ipsum has been the industrys standard dummy text ever since the 1500s, when an unknown printer took a galley of type and scrambled it to make a type specimen book. It has survived not only five centuries, but also the leap into electronic typesetting' trigger={true} />} />
        <Route path="/home" element={<Home />} />
        <Route path="/conversation" element={<Conversation />}/>
        <Route path="/login" element={<Login />}/>
        <Route path="/cadastro" element={<Signup />}/>
        <Route path="/cadastro_paciente" element={<Signup_pac />} />
        <Route path="/home_app" element={<Home_app />}/>
        <Route path="/perfil" element={<Perfil />} />
      </Routes>
    </BrowserRouter>  
  </React.StrictMode>
)
