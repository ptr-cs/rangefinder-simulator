import './App.css';
import * as React from 'react';
import 'bootstrap/dist/css/bootstrap.min.css';
import RangefinderSimulator from './components/RangefinderSimulator';

export default function App() {
  return (
    <>
    <div className="App">
      <main>
        <RangefinderSimulator/>
      </main>
    </div>
    </>
  );
}