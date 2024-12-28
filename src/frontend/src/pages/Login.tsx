import React, { useState, CSSProperties } from "react";
import { useNavigate } from "react-router-dom"; // Importa o useNavigate para redirecionamento

export default function Login() {
  // Define os tipos dos campos disponíveis
  type FieldName = "email" | "password";

  // Estado para hover em campos específicos
  const [hoverField, setHoverField] = useState<FieldName | null>(null);

  // Função para obter estilo dos inputs
  const getInputStyle = (field: FieldName): CSSProperties => ({
    width: "300px",
    border: "1px solid #001D6C",
    borderRadius: "9px",
    backgroundColor: hoverField === field ? "#D1D5DB" : "#F3F3F3", // Gray-300 no hover
    padding: "10px 16px",
    marginTop: "8px",
    transition: "background-color 0.7s ease-in-out", // Transição suave do fundo
  });

  const navigate = useNavigate(); // Inicializa o hook useNavigate

  // Função para tratar o login
  const handleLogin = (e: React.FormEvent) => {
    e.preventDefault(); // Previne o comportamento padrão do formulário
    navigate("/home"); // Redireciona o usuário para a página /home
  };

  const createSession = async () => {
    const response = await fetch("http://127.0.0.1:8000/session", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      }
    });

    console.log(response);

    if (response.ok) {
      const data = await response.json();
      console.log(data);

      // Salvar no localStorage
      localStorage.setItem("tokenGemini", data['session_id']);

      // Salvar no localStorage que o chatbot não foi iniciado
      localStorage.setItem("chatbotStarted", "false");
    }
  }

  return (
    <div className="flex flex-col justify-center items-center bg-[#F3F3F3] h-screen font-mono">
      <img src="/LoginLogo.svg" alt="" className="w-3/12" />
      <form onSubmit={handleLogin}>
        <div className="flex-1 justify-center items-center p-8 mx-auto">
          {/* Campo Usuário */}
          <div className="flex flex-col mt-8">
            <input
              type="email"
              id="email"
              placeholder="Usuário"
              style={getInputStyle("email")}
              onMouseEnter={() => setHoverField("email")}
              onMouseLeave={() => setHoverField(null)}
              className="focus:outline-none placeholder:text-[#001D6C]"
            />
            <style>
              {`
              #email::placeholder {
                transform: ${
                  hoverField === "email" ? "translateX(-5px)" : "translateX(0)"
                };
                transition: transform 0.7s ease-in-out;
              }
              `}
            </style>
          </div>

          {/* Campo Senha */}
          <div className="flex flex-col mt-4 mb-32">
            <input
              type="password"
              id="password"
              placeholder="Senha"
              style={getInputStyle("password")}
              onMouseEnter={() => setHoverField("password")}
              onMouseLeave={() => setHoverField(null)}
              className="focus:outline-none placeholder:text-[#001D6C]"
            />
            <style>
              {`
              #password::placeholder {
                transform: ${
                  hoverField === "password" ? "translateX(-5px)" : "translateX(0)"
                };
                transition: transform 0.7s ease-in-out;
              }
              `}
            </style>
          </div>

          {/* Botão de login */}
          <div className="flex justify-center mt-4">
            <button
              type="submit" // Define o tipo como submit
              className="border border-[#001D6C] placeholder-[#001D6C] hover:bg-[#001D6C] hover:text-white duration-700 rounded-[8px] bg-[#F3F3F3] px-16 py-[5px] transition-all"
              onClick={() => createSession()}
            >
              Entrar
            </button>
          </div>

          {/* Texto de criar conta */}
          <div className="flex justify-center mt-4">
            <p className="text-base font-[14px] text-black">
              <a href="/cadastro" className="underline font-condensed">
                Não tem uma conta? Clique aqui!
              </a>
            </p>
          </div>
        </div>
      </form>
    </div>
  );
}
