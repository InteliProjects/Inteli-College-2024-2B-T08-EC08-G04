import Navbar from '../components/Navbar';
import PatientCard from '../components/PatientCard';
import {
    LineChart,
    Line,
    XAxis,
    YAxis,
    CartesianGrid,
    Tooltip,
    ResponsiveContainer,
} from 'recharts';
import { useState } from 'react';
import { FiBell, FiExternalLink } from 'react-icons/fi';

export default function Perfil() {
    // Dados para o gráfico
    const data = [
        { name: 'DOM', nivel: 5, mensagem: 'Seu dia foi muito bom.' },
        { name: 'SEG', nivel: 3, mensagem: 'Dia neutro.' },
        { name: 'TER', nivel: 4, mensagem: 'Seu dia foi agradável.' },
        { name: 'QUA', nivel: 2, mensagem: 'Dia um pouco desagradável.' },
        { name: 'QUI', nivel: 1, mensagem: 'Seu dia foi muito ruim.' },
        { name: 'SEX', nivel: 3, mensagem: 'Dia neutro.' },
        { name: 'SAB', nivel: 4, mensagem: 'Seu dia foi agradável.' },
    ];

    const [popupMessage, setPopupMessage] = useState<string | null>(null);

    return (
        <div className="flex flex-col bg-[#F3F3F3] min-h-screen">
            <Navbar />

            {/* Conteúdo principal */}
            <div className="flex flex-col lg:flex-row p-6 gap-6">
                {/* Coluna de informações */}
                <div className="flex-1 lg:w-1/2">
                    <div className="relative">
                        <PatientCard
                            imageSrc="/Place.svg"
                            nome="Fernando Antônio Sampaio Cabral de Vasconcelos"
                            idade="83 anos"
                            convenio="Medlife"
                            robot_id={1}
                        />
                        {/* Ícones */}
                        <div className="absolute top-2 right-2 flex gap-3">
                            <FiExternalLink className="text-gray-700 text-2xl cursor-pointer" title="Redirecionar" />
                        </div>
                    </div>

                    {/* Últimas Ocorrências */}
                    <div className="bg-white shadow-md rounded-xl p-6 mt-6">
                        <div className="flex items-center justify-between mb-4">
                            <h2 className="text-xl font-IBM Plex Mono">Últimas ocorrências:</h2>
                            <FiBell className="text-gray-700 text-2xl cursor-pointer" title="Notificações" />
                        </div>
                        <ul className="list-disc ml-6 text-lg">
                            <li className="text-gray-700 font-IBM Plex Sans Condensed">09/11 - Alarme do remédio 3 perdido às 16:00</li>
                            <li className="text-gray-700 font-IBM Plex Sans Condensed">09/11 - Desânimo relatado às 18:00</li>
                            <li className="text-gray-700 font-IBM Plex Sans Condensed">09/11 - Queda detectada às 20:00</li>
                            <li className="text-gray-700 font-IBM Plex Sans Condensed">09/11 - Alarme do remédio 4 perdido às 22:00</li>
                            <li className="text-gray-700 font-IBM Plex Sans Condensed">09/11 - Alarme do remédio 5 perdido às 23:00</li>
                        </ul>
                    </div>
                </div>

                {/* Linha divisória */}
                <div className="w-100 w-[2px] bg-gray-300 my-100"></div>

                {/* Gráfico */}
                <div className="flex-1 lg:w-1/2 bg-white shadow-md rounded-xl p-6">
                    <h2 className="text-xl font-bold mb-4 text-center">Nível de Bem-Estar</h2>
                    <ResponsiveContainer width="100%" height={300}>
                        <LineChart
                            data={data}
                            margin={{ top: 20, right: 30, bottom: 20, left: 50 }} // Aumente a margem esquerda
                            onClick={(e) => {
                                if (e && e.activePayload) {
                                    setPopupMessage(e.activePayload[0].payload.mensagem);
                                }
                            }}
                        >
                            <CartesianGrid stroke="#e0e0e0" strokeDasharray="5 5" />
                            <XAxis dataKey="name" />
                            <YAxis
                                domain={[0, 3]}
                                tickFormatter={(tick: number) => {
                                    if (tick === 5) return 'Muito agradável';
                                    // if (tick === 4) return 'Agradável';
                                    // if (tick === 3) return 'Neutro';
                                    if (tick === 2) return 'Neutro';
                                    // if (tick === 1) return 'Muito desagradável';
                                    if (tick === 0) return 'Muito desagradável';
                                    return '';
                                }}
                                width={100} // Aumente o espaço do eixo Y
                                tickMargin={40} // Adiciona margem ao texto
                            />
                            <Tooltip />
                            {/* Apenas os pontos no gráfico */}
                            <Line
                                type="monotone"
                                dataKey="nivel"
                                stroke="transparent"
                                dot={(props) => {
                                    const { cx, cy, payload } = props; // Propriedades do ponto no gráfico
                                    const nivel = payload.nivel; // Pega o valor 'nivel' atual
                            
                                    // Define a cor com base no valor do nível
                                    let fillColor = '';
                                    if (nivel === 5) fillColor = '#006400'; // Verde escuro
                                    else if (nivel > 3 && nivel < 5) fillColor = '#90EE90'; // Verde claro
                                    else if (nivel === 3) fillColor = '#FFD700'; // Amarelo
                                    else if (nivel > 1 && nivel < 3) fillColor = '#FFA500'; // Laranja
                                    else if (nivel === 1) fillColor = '#FF0000'; // Vermelho
                            
                                    return (
                                        <circle
                                            cx={cx}
                                            cy={cy}
                                            r={8} // Tamanho das bolinhas
                                            fill={fillColor} // Aplica a cor dinâmica
                                            stroke="#fff" // Borda branca opcional
                                            strokeWidth={2}
                                        />
                                    );
                                }}
                                activeDot={{ r: 12 }} // Tamanho do ponto ativo ao clicar
                            />
                        </LineChart>
                    </ResponsiveContainer>
                    <p className="text-center text-lg text-gray-600 mt-4">Clique no círculo para exibir mais detalhes.</p>
                </div>
            </div>

            {/* Pop-up */}
            {popupMessage && (
                <div className="fixed top-0 left-0 w-full h-full flex items-center justify-center bg-black bg-opacity-50">
                    <div className="bg-white p-4 rounded shadow-md">
                        <p className="text-gray-700">{popupMessage}</p>
                        <button
                            className="mt-2 bg-blue-500 text-white px-4 py-2 rounded"
                            onClick={() => setPopupMessage(null)}
                        >
                            Fechar
                        </button>
                    </div>
                </div>
            )}
        </div>
    );
}