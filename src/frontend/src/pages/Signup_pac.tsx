import React, { useState, CSSProperties } from 'react';
import { useNavigate } from 'react-router-dom';

export default function Signup() {
    const navigate = useNavigate();

    type FieldName = 'name' | 'email' | 'cpf' | 'password' | 'password_validation' | 'QR_Code';
    const [hoverField, setHoverField] = useState<FieldName | null>(null);

    // Estado para exibir ou ocultar o modal (pop-up)
    const [isModalOpen, setIsModalOpen] = useState(false);

    // Função para fechar o modal
    const closeModal = () => setIsModalOpen(false);

    const getInputStyle = (field: FieldName): CSSProperties => ({
        width: '100%',
        maxWidth: '600px',
        border: '1px solid #001D6C',
        borderRadius: '9px',
        backgroundColor: hoverField === field ? '#BBBBBB' : '#F3F3F3',
        padding: '10px 26px',
        marginTop: '8px',
        transition: 'background-color 0.7s ease-in-out',
    });

    // Função para lidar com o clique do botão OK
    const handleOkClick = (e: React.MouseEvent<HTMLButtonElement>) => {
        e.preventDefault(); // Previne o comportamento padrão do formulário
        setIsModalOpen(true); // Abre o modal
    };

    return (
        <div className="flex flex-col justify-center items-center bg-[#F3F3F3] min-h-screen px-4 sm:px-8">
            <h1 className="text-center text-xl mb-8 font-sans text-[#001D6C]">
                Recomendamos que o responsável realize o cadastro do usuário.
            </h1>
            <h1 className="text-3xl sm:text-5xl mb-8 font-sans">Cadastre-se</h1>
            <form action="" className="w-full max-w-lg">
                <div className="flex flex-col p-4">
                    {/* Campo Nome */}
                    <input
                        type="text"
                        id="name"
                        placeholder="Nome completo"
                        style={getInputStyle('name')}
                        onMouseEnter={() => setHoverField('name')}
                        onMouseLeave={() => setHoverField(null)}
                        className="focus:outline-none placeholder:text-[#001D6C]"
                    />

                    {/* Campo Email */}
                    <input
                        type="email"
                        id="email"
                        placeholder="Email para receber notificações"
                        style={getInputStyle('email')}
                        onMouseEnter={() => setHoverField('email')}
                        onMouseLeave={() => setHoverField(null)}
                        className="focus:outline-none placeholder:text-[#001D6C] mt-4"
                    />

                    {/* Campo CPF */}
                    <input
                        type="text"
                        id="cpf"
                        placeholder="CPF"
                        style={getInputStyle('cpf')}
                        onMouseEnter={() => setHoverField('cpf')}
                        onMouseLeave={() => setHoverField(null)}
                        className="focus:outline-none placeholder:text-[#001D6C] mt-4"
                    />

                    {/* Campo criação de senha */}
                    <input
                        type="password"
                        id="password"
                        placeholder="Crie uma Senha"
                        style={getInputStyle('password')}
                        onMouseEnter={() => setHoverField('password')}
                        onMouseLeave={() => setHoverField(null)}
                        className="focus:outline-none placeholder:text-[#001D6C] mt-4"
                    />

                    {/* Campo validação de senha */}
                    <input
                        type="password"
                        id="password_validation"
                        placeholder="Confirme sua senha"
                        style={getInputStyle('password_validation')}
                        onMouseEnter={() => setHoverField('password_validation')}
                        onMouseLeave={() => setHoverField(null)}
                        className="focus:outline-none placeholder:text-[#001D6C] mt-4"
                    />

                    {/* Quadrado com botão */}
                    <div className="border-2 border-[#001D6C] flex flex-col justify-center items-center mt-8 mb-8 mx-auto w-full max-w-xs h-auto p-4">
                        <p className="text-center text-sm text-[#001D6C] mb-4">
                            Digitalize o QR Code presente no robô
                        </p>
                        <button
                            className="px-4 py-2 bg-transparent text-[#001D6C] border-2 border-[#001D6C] rounded-lg hover:bg-[#001D6C] hover:text-white transition-all"
                            onClick={() => console.log('Botão de QR Code clicado')}
                        >
                            Clique aqui
                        </button>
                    </div>

                    {/* Campo QR Code */}
                    <input
                        type="text"
                        id="QR_Code"
                        placeholder="Ou, digite o código aqui"
                        style={getInputStyle('QR_Code')}
                        onMouseEnter={() => setHoverField('QR_Code')}
                        onMouseLeave={() => setHoverField(null)}
                        className="focus:outline-none placeholder:text-[#001D6C] mt-4"
                    />

                    {/* Botão para abrir o modal */}
                    <div className="flex justify-center mt-6">
                        <button
                            type="button"
                            className="border border-[#001D6C] placeholder-[#001D6C] hover:bg-[#001D6C] hover:text-white duration-700 rounded-[7px] bg-[#F3F3F3] px-16 py-[4px] transition-all"
                            onClick={handleOkClick}
                        >
                            OK
                        </button>
                    </div>
                </div>
            </form>

            {/* Pop-up */}
            {isModalOpen && (
                <div className="fixed inset-0 bg-black bg-opacity-50 flex justify-center items-center z-50">
                    <div className="bg-white w-96 md:w-[600px] rounded-lg p-8 relative">
                        {/* Botão de fechar */}
                        <button
                            className="absolute top-4 right-4 text-gray-500 hover:text-gray-800"
                            onClick={closeModal}
                        >
                            &#10005;
                        </button>

                        <h2 className="text-2xl font-IBM Plex Sans text-center mb-4">Sobre seus dados</h2>
                        <p className="text-center text-gray-600 mb-6">
                            O J.A.R.B.A.S. armazena somente dados necessários para análise médica. Isso inclui apenas informações sobre:
                        </p>

                        {/* Checkbox items */}
                        <div className="flex flex-col space-y-4">
                            {[
                                "Suas emoções.",
                                "Suas sensações físicas.",
                                "Seus hábitos alimentares.",
                                "Medicamentos prescritos para você.",
                                "Problemas de saúde específicos.",
                                "Quem você é (nome, idade etc).",
                            ].map((text, index) => (
                                <label key={index} className="flex items-center">
                                    <input
                                        type="checkbox"
                                        className="mr-2 h-5 w-5 appearance-none border-2 border-blue-500 rounded checked:bg-blue-500 checked:border-blue-500"
                                        defaultChecked
                                    />
                                    {text}
                                </label>
                            ))}
                        </div>

                        {/* Botão para fechar o modal */}
                        <div className="flex justify-center mt-6">
                            <button
                                className="border border-[#001D6C] placeholder-[#001D6C] hover:bg-[#001D6C] hover:text-white duration-700 rounded-[7px] bg-[#F3F3F3] px-16 py-[4px] transition-all"
                                onClick={closeModal}
                            >
                                OK
                            </button>
                        </div>

                        {/* Texto adicional */}
                        <p className="text-center text-sm text-blue-600 mt-4">
                            Para saber mais sobre como nosso sistema segue a LGPD, clique aqui.
                        </p>
                    </div>
                </div>
            )}
        </div>
    );
}