import React, { useState, useEffect } from 'react';
import { db, rtdb } from '../../firebase/config';
import {
  Container,
  HourH1,
  SideInformations,
  ShowReminders,
  ShowReminderScreen,
} from './styles';

function Home() {
  const [date, setDate] = useState(new Date());
  const [nextReminder, setNextReminder] = useState('');
  const [showReminder, setShowReminder] = useState(false);
  const [actualReminder, setActualReminder] = useState('');
  const [actualTemp, setActualTemp] = useState(20);
  const alarmRef = rtdb.ref('/');
  alarmRef.on('value', (snapshot) => {
    snapshot.val();
  });

  useEffect(() => {
    const getReminders = [];
    const today = new Date();
    const rem = db
      .collection('reminders')
      .where(
        'day',
        '==',
        `${today.getUTCDate()}/${today.getUTCMonth()}/${today.getUTCFullYear()}`
      )
      .onSnapshot((querySnapshot) => {
        querySnapshot.forEach((doc) => {
          getReminders.push({ uid: doc.id, ...doc.data() });
        });
        setNextReminder(getReminders[0]);
      });
    return () => rem();
  }, []);

  useEffect(() => {
    const intervalId = setInterval(() => {
      const actualDate = new Date();
      const actualHour = `${actualDate.getHours()}:${actualDate.getMinutes()}`;
      let alarmVal = 0;
      alarmRef.on('value', (snapshot) => {
        alarmVal = snapshot.val()['ALARM'];
        setActualTemp(snapshot.val()['TEMP_AMB']);
      });
      setDate(actualDate);
      if (alarmVal === 0) {
        setShowReminder(false);
      } else {
        db.collection('reminders').doc(nextReminder?.uid).delete();
        setNextReminder('');
      }
      if (actualHour === nextReminder?.hour) {
        setShowReminder(true);
        setActualReminder(nextReminder.description);
        alarmRef.update({ ALARM: 1 });
      }
    }, 200);
    return () => clearInterval(intervalId);
  }, [nextReminder, alarmRef]);

  return (
    <ShowReminders to='/reminders'>
      <Container>
        {showReminder ? (
          <ShowReminderScreen>{actualReminder}</ShowReminderScreen>
        ) : (
          <>
            <SideInformations>
              Temperatura ambiente: <b>{actualTemp}°C</b>
            </SideInformations>
            <div>
              <HourH1>
                {date.getHours()}:{date.getMinutes()}
              </HourH1>
              <span>
                <b>
                  {date.getDate()}/{date.getUTCMonth() + 1}/
                  {date.getUTCFullYear()}
                </b>
              </span>
            </div>
            <SideInformations>
              {nextReminder !== '' && nextReminder !== undefined ? (
                <span>
                  Proximo lembrete: <b>{nextReminder?.hour}</b>
                </span>
              ) : (
                <span>Não há lembretes hoje</span>
              )}
            </SideInformations>
          </>
        )}
      </Container>
    </ShowReminders>
  );
}

export default Home;
