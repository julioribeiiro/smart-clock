import './App.css';
import Header from './components/header';
import Reminders from './pages/Reminders';
import FormsPage from './pages/FormsPage';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import Home from './pages/Home';

function App() {
  return (
    <Router>
      <div className='App'>
        <Header />
        <Routes>
          <Route path='/' exact element={<Home />} />
          <Route path='/reminders' element={<Reminders />} />
          <Route path='/forms' element={<FormsPage />} />
        </Routes>
      </div>
    </Router>
  );
}

export default App;
