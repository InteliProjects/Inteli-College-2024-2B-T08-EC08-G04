type ButtonProps = {
  label: string; // Texto do botão
  onClick: () => void; // Função chamada ao clicar
  customStyles?: string; // Estilos adicionais personalizados (opcional)
};

export function Button_entrar({ label, onClick, customStyles }: ButtonProps) {
  return (
    <button
      className={`bg-[#0043CE] text-white py-2 px-6 rounded-xl w-[181px] h-[54px] opacity-100 hover:brightness-110 transition-all focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-[#0043CE] ${customStyles}`}
      onClick={onClick}
    >
      {label}
    </button>
  );
}

export function Button_cadastrar({ label, onClick, customStyles }: ButtonProps) {
  return (
    <button
      className={`bg-[#001D6C] text-white text-lg py-2 px-6 rounded-xl w-[181px] h-[54px] opacity-100 hover:brightness-110 transition-all focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-[#001D6C] ${customStyles}`}
      onClick={onClick}
    >
      {label}
    </button>
  );
}
