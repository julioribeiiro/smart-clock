import Firebase from 'firebase/compat/app';
import 'firebase/compat/firestore';
import "firebase/compat/firestore";
import 'firebase/compat/database';

const firebaseConfig = {
    apiKey: 'AIzaSyAPCY5rW0thEvDbRoArQ3TOtcvJxc1Enhs',
    authDomain: 'smart-clock-a7cc8.firebaseapp.com',
    projectId: 'smart-clock-a7cc8',
    storageBucket: 'smart-clock-a7cc8.appspot.com',
    messagingSenderId: '892507659091',
    appId: '1:892507659091:web:c14bd4a10d9d375c0b5fa6',
  };

  const firebase = Firebase.initializeApp(firebaseConfig);

  const db = firebase.firestore();
  const rtdb = firebase.database();

  export { firebase, db, rtdb }