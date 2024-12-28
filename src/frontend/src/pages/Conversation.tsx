import React, { useState } from 'react';
import Card from '../components/Card';
import TalkButton from '../components/TalkButton';
import TextInput from '../components/TextInput';
import BatteryIcon from '../assets/battery.svg';
import ResponseOverlay from '../components/ResponseOverlay';

export default function Conversation() {
  const [inputValue, setInputValue] = useState(''); // Estado para gerenciar o valor do TextInput
  const [responseService, setResponseService] = useState<string | null>(null);
  const [trigger, setTrigger] = useState(false);

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setInputValue(event.target.value); // Atualiza o estado ao digitar
  };

  const handleKeyPress = (event: React.KeyboardEvent<HTMLInputElement>) => {
    if (event.key === 'Enter') {
      console.log('Enter pressionado: ', inputValue); // Apenas exibe no console por enquanto
      setInputValue(''); // Limpa o campo após o envio
    }
  };

  return (
    <div className="bg-[#F3F3F3] min-h-screen flex flex-col items-center p-4 space-y-6 relative">
      {/* Card no topo */}
      <div className="w-full max-w-md mt-1 z-50 mb-16">
        <Card>
          <div className="h-1/6 flex items-center justify-between">
            <p className="text-gray-800 text-lg sm:text-xl">Status do robô:</p>
            <div className="flex items-center space-x-1">
              <span className="w-3 h-3 sm:w-4 sm:h-4 bg-green-500 rounded-full"></span>
              <span className="text-gray-600 text-base sm:text-lg font-bold">Operando</span>
            </div>
          </div>
          <div className="flex items-center justify-between mt-4">
            <p className="text-gray-800 text-lg sm:text-xl">Status da bateria:</p>
            <div className="flex items-center space-x-1">
              <img src={BatteryIcon} alt="Bateria" className="w-4 h-4 sm:w-5 sm:h-5" />
              <span className="text-gray-600 text-base sm:text-lg font-bold">78%</span>
            </div>
          </div>
        </Card>
      </div>

      <div>
        {/* Conteúdo centralizado */}
        <div className={`flex flex-col items-center justify-center flex-grow space-y-6 ${trigger ? 'hidden' : ''} z-10`}>
          <p className="px-8 tracking-tighter text-center text-2xl sm:text-2xl px-5 font-light font-sans">
            Clique e segure para falar com o robô
          </p>
          <TalkButton setResponseService={setResponseService} setTrigger={setTrigger} />
          {/* Campo de texto funcional */}
          <div className="w-full max-w-md px-16">
            <TextInput
              placeholder="Ou digite aqui"
              value={inputValue} // Conecta ao estado
              onChange={handleInputChange} // Atualiza o estado ao digitar
              onKeyDown={handleKeyPress} // Detecta a tecla Enter
            />
          </div>
        </div>

        {/* Resposta do LLM */}
        {responseService && (
            console.log(responseService),
            <ResponseOverlay text={responseService} trigger={trigger} />
        )}
      </div>
    </div>
  );
}
