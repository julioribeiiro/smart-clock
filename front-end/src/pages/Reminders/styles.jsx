import styled from 'styled-components';
import { Link } from 'react-router-dom';

export const Container = styled.div`
  display: flex;
  align-items: center;
  justify-content: flex-start;
  flex-direction: column;
  width: 800px;
  height: auto;
  background-color: #e6e3e3;
`;

export const AddNewButton = styled(Link)`
  color: white;
  font-weight: bold;
  background-color: #000;
  width: 100%;
  padding: 15px 0;
  border-radius: 10px;
  text-decoration: none;
`;

export const UpperDiv = styled.div`
  display: flex;
  align-items: center;
  justify-content: center;
  position: relative;
  width: 100%;
`;

export const BackLink = styled(Link)`
  position: absolute;
  left: 30px;
  font-size: 2rem;
  background-color: transparent;
  border: 0;
  cursor: pointer;
  text-decoration: none;
  color: #000;
`;
