/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      fontFamily: {
        mono: ['"IBM Plex Mono"', 'monospace'],
        sans: ['"IBM Plex Sans"', 'sans-serif'], // Adiciona IBM Plex Sans
        condensed: ['"IBM Plex Sans Condensed"', 'sans-serif'], // Adiciona IBM Plex Sans Condensed
      },
    },
  },
  plugins: [],
}