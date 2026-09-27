import '@fontsource/inter/400.css';
import '@fontsource/inter/500.css';
import '@fontsource/jetbrains-mono/500.css';
import './styles/nocturne.css';
import './styles/theme.css';
import { mount } from 'svelte';
import App from './components/App.svelte';

mount(App, { target: document.getElementById('app') });
