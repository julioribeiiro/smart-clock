import React, { useState } from 'react';
import { Card, CardContent, TextField, Grid, Button } from '@mui/material';
import { LocalizationProvider, DateTimePicker } from '@mui/lab';
import AdapterDateFns from '@mui/lab/AdapterDateFns';
import { db } from '../../firebase/config';
import { useNavigate } from 'react-router-dom';

function FormsPage() {
  const [dateTime, setDateTime] = useState(new Date());
  const [description, setDescription] = useState('');
  let navigate = useNavigate();

  const storeReminder = (e) => {
    e.stopPropagation();
    const date =
      dateTime.getDate() +
      '/' +
      dateTime.getUTCMonth() +
      '/' +
      dateTime.getUTCFullYear();
    const hour = dateTime.getHours() + ':' + dateTime.getUTCMinutes();

    const data = { description: description, day: date, hour: hour };

    db.collection('reminders').add(data);
    navigate('/');
  };

  return (
    <div style={{ width: '800px' }}>
      <h1>Crie um lembrete</h1>
      <Card>
        <CardContent>
          <Grid xs={12} sm={0} item>
            <h3>Selecione a data e hora do lembrete</h3>
            <LocalizationProvider dateAdapter={AdapterDateFns}>
              <DateTimePicker
                renderInput={(props) => <TextField {...props} />}
                label='Data e Hora'
                value={dateTime}
                onChange={(newValue) => {
                  setDateTime(newValue);
                }}
              />
            </LocalizationProvider>
          </Grid>
          <Grid container spacing={1}>
            <Grid xs={12} sm={12} item>
              <h3>Escreva a atividade</h3>
              <TextField
                label='Tarefa'
                rows={2}
                multiline
                placeholder='Escreva a tarefa'
                fullWidth
                onChange={(e) => {
                  setDescription(e.target.value);
                }}
              />
            </Grid>
          </Grid>
        </CardContent>
        <Button
          variant='contained'
          fullWidth
          onClick={storeReminder}
          style={{ background: 'gray' }}
        >
          Criar
        </Button>
      </Card>
    </div>
  );
}

export default FormsPage;
