export default function Navbar() {
  return (
    <div className="flex justify-between items-center px-12 py-[12px] bg-white">
      <div>
        <a href="/home">
          <img src="/Home.svg" alt="Logo" className="w-8" />
        </a>
      </div>
      <div>
        <a href="/login">
          <img src="/Login.svg" alt="Logo" className="w-8" />
        </a>
      </div>
    </div>
  );
}
