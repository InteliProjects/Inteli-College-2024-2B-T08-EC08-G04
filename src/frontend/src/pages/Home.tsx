import Navbar from '../components/Navbar';
import PatientCard from '../components/PatientCard';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faSearch } from '@fortawesome/free-solid-svg-icons';

export default function Home() {
  return (
    <div className="flex flex-col bg-[#F3F3F3] min-h-screen">
      <Navbar />
      <div className="w-4/5 p-8 mx-auto">
        <h1 className="text-3xl font-bold text-start mt-8">Olá, Dra. Fulane!</h1>
        <h2 className="text-lg font-medium text-start mt-4">Selecione um dos seus pacientes:</h2>
        
        {/* Search Bar */}
        <div className="flex justify-end mt-4 mb-8">
          <div className="relative w-1/3">
            <input
              type="text"
              placeholder="Pesquise aqui"
              className="bg-[#E1E5F1] rounded-2xl px-4 py-[4px] w-full pr-10"
            />
            <div className="absolute inset-y-0 right-0 flex items-center pr-3 pointer-events-none">
              {/* Search Icon */}
              <FontAwesomeIcon icon={faSearch} className="text-[#0043CE] font-thin w-4 h-4" />
            </div>
          </div>
        </div>

        {/* Patient Cards */}
        <a href='/perfil'>
        <PatientCard
          imageSrc="/Place.svg"
          nome="Fulano de Tal"
          idade="25 anos"
          convenio="Unimed"
          robot_id={1}
        />
        </a>
        <PatientCard
          imageSrc="/Place.svg"
          nome="Fulano de Tal"
          idade="25 anos"
          convenio="Unimed"
          robot_id={1}
        />
      </div>
    </div>
  );
}
