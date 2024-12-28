import React, { useState, useRef, useEffect } from "react";
import MicrophoneIcon from "../assets/microphone.svg"; // Verifique o caminho do arquivo .svg

const TalkButton: React.FC<{ setResponseService: (response: string | null) => void, setTrigger: (trigger: boolean) => void }> = ({ setResponseService, setTrigger }) => {
  const [isRecording, setIsRecording] = useState(false);
  const [audioUrl, setAudioUrl] = useState<string | null>(null);
  const [transcription, setTranscription] = useState<string | null>(null);
  const [ResponseAudio, setResponseAudio] = useState<string | null>(null);
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const audioChunks = useRef<Blob[]>([]);

  useEffect(() => {
    if (ResponseAudio) {
      const audio = new Audio(`data:audio/mp3;base64,${ResponseAudio}`);
      setTrigger(true);
      audio.play();
      audio.onended = () => {
        setTrigger(false);
      };
    }
  }, [ResponseAudio, setTrigger]);

  const handleStartRecording = async () => {
    setIsRecording(true);
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      const mediaRecorder = new MediaRecorder(stream, {
        mimeType: "audio/webm;codecs=opus",
      });
      mediaRecorderRef.current = mediaRecorder;

      mediaRecorder.ondataavailable = (event) => {
        audioChunks.current.push(event.data);
      };

      mediaRecorder.onstop = async () => {
        const audioBlob = new Blob(audioChunks.current, { type: "audio/webm" });
        const audioUrl = URL.createObjectURL(audioBlob);
        setAudioUrl(audioUrl);
        audioChunks.current = [];

        // Converte o áudio para WAV antes de enviar
        const wavBlob = await convertWebmToWav(audioBlob);
        await sendAudioToServer(wavBlob);
      };

      mediaRecorder.start();
    } catch (error) {
      console.error("Erro ao acessar o microfone:", error);
      setIsRecording(false);
    }
  };

  const handleStopRecording = () => {
    setIsRecording(false);
    if (mediaRecorderRef.current && mediaRecorderRef.current.state !== "inactive") {
      mediaRecorderRef.current.stop();
    }
  };

  const sendAudioToServer = async (audioBlob: Blob) => {
    try {
      const formData = new FormData();
      formData.append("file", audioBlob, "audio.wav"); // Nome do campo do arquivo

      console.log("Enviando áudio para o backend...");

      // Pegar o tokenGemini no localStorage

      const tokenGemini = localStorage.getItem("tokenGemini");
      const chatbotStarted = localStorage.getItem("chatbotStarted");

      if (!tokenGemini) {
        throw new Error("Token não encontrado");
      }

      console.log("Token Gemini: ", tokenGemini);

      const response = await fetch(`http://127.0.0.1:8000/speech-to-text?session_id_gemini=${tokenGemini}&chatbotStarted=${chatbotStarted}`, {
        method: "POST",
        body: formData,
      });

      // Salvar no localSrtage
      localStorage.setItem("chatbotStarted", "true");

      console.log("Resposta do backend:", response);

      if (!response.ok) {
        const errorText = await response.text();
        throw new Error(`Erro do backend: ${response.status} - ${errorText}`);
      }

      const data = await response.json();
      console.log("Resposta do backend:", data);
      setTranscription(data.transcript);
      
      console.log("Transcrição:", data.transcript);
      console.log("Resposta:", data.response);
      setResponseService(data.response);

      setResponseAudio(data.audio_file);

    } catch (error) {
      console.error("Erro ao processar o áudio:", error);
    }
  };

  // Função para converter WebM para WAV
  const convertWebmToWav = async (audioBlob: Blob) => {
    const audioContext = new AudioContext();
    const arrayBuffer = await audioBlob.arrayBuffer();
    const audioBuffer = await audioContext.decodeAudioData(arrayBuffer);

    return audioBufferToWav(audioBuffer);
  };

  // Helper para codificar WAV
  const audioBufferToWav = (audioBuffer: AudioBuffer) => {
    const numOfChannels = audioBuffer.numberOfChannels;
    const length = audioBuffer.length * numOfChannels * 2 + 44;
    const buffer = new ArrayBuffer(length);
    const view = new DataView(buffer);

    const channels: Float32Array[] = [];
    for (let i = 0; i < numOfChannels; i++) {
      channels.push(audioBuffer.getChannelData(i));
    }

    let offset = 0;

    const writeString = (s: string) => {
      for (let i = 0; i < s.length; i++) {
        view.setUint8(offset++, s.charCodeAt(i));
      }
    };

    // WAV Header
    writeString("RIFF");
    view.setUint32(offset, length - 8, true);
    offset += 4;
    writeString("WAVE");
    writeString("fmt ");
    view.setUint32(offset, 16, true);
    offset += 4;
    view.setUint16(offset, 1, true);
    offset += 2;
    view.setUint16(offset, numOfChannels, true);
    offset += 2;
    view.setUint32(offset, audioBuffer.sampleRate, true);
    offset += 4;
    view.setUint32(offset, audioBuffer.sampleRate * 2 * numOfChannels, true);
    offset += 4;
    view.setUint16(offset, numOfChannels * 2, true);
    offset += 2;
    view.setUint16(offset, 16, true);
    offset += 2;
    writeString("data");
    view.setUint32(offset, length - offset - 4, true);
    offset += 4;

    // Interleave channels
    for (let i = 0; i < audioBuffer.length; i++) {
      for (let channel = 0; channel < numOfChannels; channel++) {
        const sample = Math.max(-1, Math.min(1, channels[channel][i]));
        view.setInt16(offset, sample < 0 ? sample * 0x8000 : sample * 0x7FFF, true);
        offset += 2;
      }
    }

    return new Blob([buffer], { type: "audio/wav" });
  };

  return (
    <div className="flex flex-col items-center space-y-6">
      {/* Botão de gravação */}
      <button
        aria-label={isRecording ? "Parar gravação" : "Iniciar gravação"}
        onMouseDown={handleStartRecording}
        onMouseUp={handleStopRecording}
        onTouchStart={handleStartRecording}
        onTouchEnd={handleStopRecording}
        className={`w-52 h-52 sm:w-64 sm:h-64 lg:w-72 lg:h-72 flex justify-center items-center rounded-full shadow-lg ${
          isRecording ? "bg-green-500" : "bg-blue-600"
        } transition-colors duration-300`}
      >
        <img
          src={MicrophoneIcon}
          alt="Microfone"
          className="w-20 h-20 sm:w-24 sm:h-24 lg:w-28 lg:h-28"
        />
      </button>

    </div>
  );
};

export default TalkButton;
