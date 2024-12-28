import React from 'react';

interface CardProps {
  imageSrc: string;
  nome: string;
  idade: string;
  convenio: string;
  robot_id: number;
}

const Card: React.FC<CardProps> = ({ imageSrc, nome, idade, convenio, robot_id }) => {
  return (
    <div className="flex bg-white shadow-md rounded-xl overflow-hidden aspect-[5/1] mb-8">
      <div className="w-1/5 h-full">
        <img src={imageSrc} className="object-cover h-full w-full" alt="Card Image" />
      </div>
      <div className="flex flex-col w-2/3 p-4 justify-center">
        <div className="mt-2 text-md flex">
          <span className="font-bold w-24">Nome:</span>
          <span className="ml-4">{nome}</span>
        </div>
        <div className="mt-2 text-md flex">
          <span className="font-bold w-24">Idade:</span>
          <span className="ml-4">{idade}</span>
        </div>
        <div className="mt-2 text-md flex">
          <span className="font-bold w-24">Convênio:</span>
          <span className="ml-4">{convenio}</span>
        </div>
        <div className="mt-2 text-md flex">
          <span className="font-bold w-24">Robot ID:</span>
          <span className="ml-4">{robot_id}</span>
        </div>
      </div>
    </div>
  );
};

export default Card;
