import React, { useEffect, useState } from 'react';
import { Container, AddNewButton, UpperDiv, BackLink } from './styles';
import NotificationsTable from '../../components/NotificationsTable';
import { db } from '../../firebase/config';

function Reminders() {
  const [loading, setLoading] = useState(true);
  const [reminders, setReminders] = useState([]);

  useEffect(() => {
    const getReminders = [];
    const rem = db.collection('reminders').onSnapshot((querySnapshot) => {
      querySnapshot.forEach((doc) => {
        getReminders.push({ uid: doc.id, ...doc.data() });
      });
      setReminders(getReminders);
      setLoading(false);
    });
    return () => rem();
  }, [reminders]);

  if (loading) {
    return <h1>loading firebase data</h1>;
  }

  return (
    <Container>
      <UpperDiv>
        <BackLink to='/'>&larr;</BackLink>
        <h1>Lembretes registrados:</h1>
      </UpperDiv>
      <NotificationsTable rows={reminders} />
      <AddNewButton to='/forms'>Adicionar Lembrete</AddNewButton>
    </Container>
  );
}

export default Reminders;
