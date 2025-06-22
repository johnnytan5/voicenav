let audioContext, pcmNode;
let initialized = false;
let mediaStream, mediaRecorder, recordedChunks = [];
let socket = io.connect(location.origin);
let listening = false;
let wasListeningBeforeTalk = false;

function prependLog(tabId, newText) {
  const tab = document.getElementById(tabId);
  if (tab) {
    const pre = tab.querySelector('pre');
    if (pre) {
      pre.textContent = newText + "\n" + pre.textContent;
      pre.scrollTop = 0; // scroll to top
    }
  }
}
 const tabs = {};
 let currentTab = null;

 function switchTab(tabName) {
   if (currentTab) document.getElementById(`tab-${currentTab}`).classList.remove('active');
   document.getElementById(`tab-${tabName}`).classList.add('active');
   currentTab = tabName;
 }


 socket.on('ros_log', (data) => {
   const node = data.node;
   const message = `[${new Date(data.timestamp * 1000).toLocaleTimeString()}] ${data.msg}\n`;


   if (!tabs[node]) {
     tabs[node] = true;
     const pre = document.createElement('pre');
     pre.id = `tab-${node}`;
     pre.className = 'tab';
     pre.textContent = '';
     document.getElementById('tabs').appendChild(pre);


     const btn = document.createElement('button');
     btn.innerText = node;
     btn.onclick = () => switchTab(node);
     document.getElementById('tab-buttons').appendChild(btn);


     switchTab(node);
   }


   const pre = document.getElementById(`tab-${node}`);
   pre.textContent = message + pre.textContent;
   pre.scrollTop = 0;
   prependLog(data, message);

 });



function startTalk() {
  wasListeningBeforeTalk = listening;  // Save before we toggle
  if (listening) toggleListen();       // Turn it off while talking

  recordedChunks = [];

  navigator.mediaDevices.getUserMedia({ audio: true }).then(stream => {
    mediaStream = stream;
    mediaRecorder = new MediaRecorder(stream, { mimeType: 'audio/webm' });

    mediaRecorder.ondataavailable = (e) => {
      if (e.data.size > 0) {
        recordedChunks.push(e.data);
      }
    };

    mediaRecorder.start();
    console.log("[Talk] Recording started.");
  }).catch(err => console.error("Mic access error:", err));
}


function stopTalk() {
  if (!mediaRecorder || mediaRecorder.state === 'inactive') return;

  mediaRecorder.onstop = () => {
    const fullBlob = new Blob(recordedChunks, { type: 'audio/webm' });
    fullBlob.arrayBuffer().then(buffer => {
      socket.emit('audio_chunk', buffer);
      console.log("[Talk] Sent recording:", buffer.byteLength, "bytes");

      // Restore listen state if it was ON before
      if (!listening && wasListeningBeforeTalk) {
        toggleListen();  // 🔁 turn it back on
      }
    });
  };

  mediaRecorder.stop();
  mediaStream.getTracks().forEach(t => t.stop());
  console.log("[Talk] Recording stopped.");
}


async function initAudio() {
  audioContext = new (window.AudioContext || window.webkitAudioContext)();
  await audioContext.audioWorklet.addModule('/static/pcm-processor.js');
  pcmNode = new AudioWorkletNode(audioContext, 'pcm-resampler');
  pcmNode.connect(audioContext.destination);

  socket = io.connect(location.origin);

  socket.on('robot_audio', rawBytes => {
    if (listening) {
      pcmNode.port.postMessage(rawBytes, [rawBytes]);
    }
  });

  console.log("AudioWorklet initialized.");
}

// On page load
document.addEventListener('DOMContentLoaded', async () => {
  await initAudio();
  initialized = true;
  updateListenButton();
});

// Listen toggle logic
async function toggleListen() {
  listening = !listening;
  if (audioContext.state === 'suspended') {
    await audioContext.resume();
  }

  document.getElementById('listen-btn').innerText = listening ? '🔊 Listen: ON' : '🔇 Listen: OFF';
  if (socket) {
    socket.emit('toggle_listen', listening);
  }

}
function updateListenButton() {
  const btn = document.getElementById('listen-btn');
  btn.innerText = listening ? '🔊 Listen: ON' : '🔇 Listen: OFF';
}
