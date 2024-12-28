import React, { useEffect, useState } from "react";
import AudioIcon from "../assets/audio.svg";

interface ResponseOverlayProps {
  text: string;
  trigger: boolean;
}

const ResponseOverlay: React.FC<ResponseOverlayProps> = ({ text, trigger }) => {
  const [visible, setVisible] = useState(false);

  useEffect(() => {
    setVisible(trigger);
  }, [trigger]);

  return (
    visible && (
      <div className="absolute inset-0 z-40">

        <div className='absolute z-60 bg-[#F3F3F3] h-screen flex flex-col justify-between items-center relative p-4'>
          
          <div className="flex flex-col justify-between space-y-4 items-center">
            
              <div className='relative mt-36 flex justify-center items-center'>
              <div className='absolute bg-[#0043CE] w-36 h-36 sm:w-56 sm:h-56 lg:w-68 lg:h-68 rounded-full animate-pulse'>

              </div>
                <div className='relative bg-[#0043CE] w-32 h-32 sm:w-52 sm:h-52 lg:w-64 lg:h-64 flex justify-center items-center rounded-full shadow-lg'>
                  <img src={AudioIcon} alt='Microfone' className='w-16 h-16 sm:w-24 sm:h-24 lg:w-28 lg:h-28' />
                </div>
              </div>

            <p className="px-8 tracking-tighter text-center text-2xl sm:text-2xl px-5 font-light font-sans">
              O robô está respondendo
            </p>

          </div>

          <div className="mt-4 flex flex-col justify-center items-center h-full">
            <div
              className="w-fit flex items-center justify-center p-1"
              style={{
                background: "linear-gradient(to bottom right, #8D9ABB, #f3f3f3 50%, #8D9ABB)",
              }}
            >
              <div className="bg-[#F3F3F3] flex items-center justify-center p-4">
              <p className="break-normal font-normal font-mono text-center text-m">
                {typeof text === "object" ? text.text : text}
              </p>
              </div>
            </div>
          </div>
        </div>
      </div>
    )
  );
};

export default ResponseOverlay;
