import styled from 'styled-components';
import { Link } from 'react-router-dom';

export const Container = styled.div`
  background-color: lightgray;
  display: flex;
  flex-direction: column;
  justify-content: space-between;
  align-items: center;
  width: 800px;
  height: 400px;
`;

export const HourH1 = styled.h1`
  font-size: 5rem;
  font-weight: 500;
  font-family: 'Orbitron', sans-serif;
`;

export const SideInformations = styled.span`
  font-size: 1.2rem;
  margin: 5px;
`;

export const ShowReminders = styled(Link)`
  text-decoration: none;
  color: #000;
`;

export const ShowReminderScreen = styled.div`
  display: flex;
  justify-content: center;
  align-items: center;
  width: 100%;
  height: 100%;
  margin: 0 30px;
  font-size: 3rem;
  font-weight: 500;
`;
