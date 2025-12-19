// Centralized Firebase init + exports (CDN, modular SDK)
import { firebaseConfig } from './firebaseConfig.js';
import { initializeApp } from "https://www.gstatic.com/firebasejs/10.12.2/firebase-app.js";
import {
  getDatabase, ref, onValue, set, push, serverTimestamp
} from "https://www.gstatic.com/firebasejs/10.12.2/firebase-database.js";
import {
  getAuth, signInAnonymously, onAuthStateChanged
} from "https://www.gstatic.com/firebasejs/10.12.2/firebase-auth.js";

const app = initializeApp(firebaseConfig);
const db = getDatabase(app);
const auth = getAuth(app);

// Disable demo mode for production
const isDemo = false;

export {
  app, db, auth,
  ref, onValue, set, push, serverTimestamp,
  signInAnonymously, onAuthStateChanged,
  isDemo
};
