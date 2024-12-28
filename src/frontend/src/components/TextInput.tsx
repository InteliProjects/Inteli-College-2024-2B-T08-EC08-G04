import React from 'react';

interface TextInputProps {
  placeholder?: string;
  value: string;
  onChange: (event: React.ChangeEvent<HTMLInputElement>) => void;
  onKeyDown?: (event: React.KeyboardEvent<HTMLInputElement>) => void;
}

const TextInput: React.FC<TextInputProps> = ({
  placeholder,
  value,
  onChange,
  onKeyDown,
}) => {
  return (
    <input
      type="text"
      className="w-full p-2 rounded-md border ring-1 ring-[#001D6C] bg-[#F3F3F3] text-gray-800 text-center"
      placeholder={placeholder}
      value={value}
      onChange={onChange}
      onKeyDown={onKeyDown}
      style={{ textAlign: 'center' }} // Centralize the placeholder text
    />
  );
};

export default TextInput;