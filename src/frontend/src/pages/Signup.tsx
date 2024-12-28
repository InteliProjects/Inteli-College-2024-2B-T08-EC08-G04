import React, { useState, CSSProperties } from 'react';
import { useNavigate } from 'react-router-dom';

export default function Signup() {
    const navigate = useNavigate();

    // Define os tipos dos campos disponíveis
    type FieldName = 'name' | 'email' | 'cpf' | 'crm' | 'password';

    // Estado para hover em campos específicos
    const [hoverField, setHoverField] = useState<FieldName | null>(null);

    // Função para obter estilo dos inputs
    const getInputStyle = (field: FieldName): CSSProperties => ({
        width: '600px',
        border: '1px solid #001D6C',
        borderRadius: '9px',
        backgroundColor: hoverField === field ? '#BBBBBB' : '#F3F3F3', // Gray-300 no hover
        padding: '10px 26px',
        marginTop: '8px',
        transition: 'background-color 0.7s ease-in-out', // Transição suave do fundo
    });

    return (
        <div className="flex flex-col justify-center items-center bg-[#F3F3F3] h-screen">
            <h1 className="text-5xl mb-8 font-sans">Cadastre-se</h1>
            <form action="">
                <div className="flex-1 justify-center items-center p-8 mx-auto font-mono">
                    {/* Campo Nome */}
                    <div className="flex flex-col mt-8">
                        <input
                            type="text"
                            id="name"
                            placeholder="Nome completo"
                            style={getInputStyle('name')}
                            onMouseEnter={() => setHoverField('name')}
                            onMouseLeave={() => setHoverField(null)}
                            className="focus:outline-none placeholder:text-[#001D6C]"
                        />
                        <style>
                            {`
                            #name::placeholder {
                                transform: ${hoverField === 'name' ? 'translateX(-5px)' : 'translateX(0)'};
                                transition: transform 0.7s ease-in-out;
                            }
                            `}
                        </style>
                    </div>

                    {/* Campo Email */}
                    <div className="flex flex-col mt-4">
                        <input
                            type="email"
                            id="email"
                            placeholder="Email"
                            style={getInputStyle('email')}
                            onMouseEnter={() => setHoverField('email')}
                            onMouseLeave={() => setHoverField(null)}
                            className="focus:outline-none placeholder:text-[#001D6C]"
                        />
                        <style>
                            {`
                            #email::placeholder {
                                transform: ${hoverField === 'email' ? 'translateX(-5px)' : 'translateX(0)'};
                                transition: transform 0.7s ease-in-out;
                            }
                            `}
                        </style>
                    </div>

                    {/* Campo CPF */}
                    <div className="flex flex-col mt-4">
                        <input
                            type="text"
                            id="cpf"
                            placeholder="CPF"
                            style={getInputStyle('cpf')}
                            onMouseEnter={() => setHoverField('cpf')}
                            onMouseLeave={() => setHoverField(null)}
                            className="focus:outline-none placeholder:text-[#001D6C]"
                        />
                        <style>
                            {`
                            #cpf::placeholder {
                                transform: ${hoverField === 'cpf' ? 'translateX(-5px)' : 'translateX(0)'};
                                transition: transform 0.7s ease-in-out;
                            }
                            `}
                        </style>
                    </div>

                    {/* Campo CRM */}
                    <div className="flex flex-col mt-4">
                        <input
                            type="text"
                            id="crm"
                            placeholder="CRM"
                            style={getInputStyle('crm')}
                            onMouseEnter={() => setHoverField('crm')}
                            onMouseLeave={() => setHoverField(null)}
                            className="focus:outline-none placeholder:text-[#001D6C]"
                        />
                        <style>
                            {`
                            #crm::placeholder {
                                transform: ${hoverField === 'crm' ? 'translateX(-5px)' : 'translateX(0)'};
                                transition: transform 0.7s ease-in-out;
                            }
                            `}
                        </style>
                    </div>

                    {/* Campo Senha */}
                    <div className="flex flex-col mt-4 mb-20">
                        <input
                            type="password"
                            id="password"
                            placeholder="Crie uma Senha"
                            style={getInputStyle('password')}
                            onMouseEnter={() => setHoverField('password')}
                            onMouseLeave={() => setHoverField(null)}
                            className="focus:outline-none placeholder:text-[#001D6C]"
                        />
                        <style>
                            {`
                            #password::placeholder {
                                transform: ${hoverField === 'password' ? 'translateX(-5px)' : 'translateX(0)'};
                                transition: transform 0.7s ease-in-out;
                            }
                            `}
                        </style>
                    </div>

                    {/* Botões */}
                    <div className="flex flex-row justify-between">
                        <div className="flex justify-center mt-4">
                            <button
                                onClick={() => navigate('/login')}
                                className="border border-[#DA1E28] placeholder-[#001D6C] hover:bg-[#DA1E28] hover:text-white duration-700 rounded-[7px] bg-[#F3F3F3] px-14 py-[4px] transition-all"
                            >
                                Cancelar
                            </button>
                        </div>

                        <div className="flex justify-center mt-4">
                            <button className="border border-[#001D6C] placeholder-[#001D6C] hover:bg-[#001D6C] hover:text-white duration-700 rounded-[7px] bg-[#F3F3F3] px-16 py-[4px] transition-all">
                                Entrar
                            </button>
                        </div>
                    </div>
                </div>
            </form>
        </div>
    );
}
