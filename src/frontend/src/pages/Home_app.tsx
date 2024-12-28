import React from "react";
import { Button_entrar, Button_cadastrar } from "../components/Button_2";

export default function Signup_pac() {
  return (
    <div className="flex flex-col justify-center items-center bg-[#F3F3F3] h-screen font-sans">
      {/* Logo */}
      <div className="flex justify-center items-center">
        <img src="/LoginLogo.svg" alt="Logo" className="w-2/3" />
      </div>

      {/* Texto de boas-vindas */}
      <h2 className="text-3xl mt-16">Bem-vinde!</h2>

      {/* Botões */}
      <div className="mt-8 flex flex-col items-center space-y-4">
        <Button_entrar
          label="Entrar"
          onClick={() => console.log("Entrar clicado")}
        />
        <Button_cadastrar
          label="Cadastrar"
          onClick={() => console.log("Cadastrar clicado")}
        />
      </div>
    </div>
  );
}
