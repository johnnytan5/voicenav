class PCMResamplerProcessor extends AudioWorkletProcessor {
  constructor() {
    super();
    this.inBuffer = new Int16Array(0);
    this.inIndex  = 0;      
    this.outBuffer = [];
    this.port.onmessage = (event) => {
      const chunk = new Int16Array(event.data);
      // append new chunk to inBuffer
      const tmp = new Int16Array(this.inBuffer.length + chunk.length);
      tmp.set(this.inBuffer);
      tmp.set(chunk, this.inBuffer.length);
      this.inBuffer = tmp;
    };

    // calculate the ratio once
    this.inputRate  = 16000;
    this.outputRate = sampleRate;            
    this.ratio      = this.inputRate / this.outputRate;
    this.started    = false;
  }

  process(inputs, outputs) {
    const out = outputs[0][0]; 
    const needed = out.length;

    // Jitter buffer: wait until we have at least N ms of data
    const minSamples = this.outputRate * 0.001; 
    if (!this.started) {
      const availableSrcSamples = this.inBuffer.length - this.inIndex;
      const availableOut = availableSrcSamples / this.ratio;
      if (availableOut < minSamples) {
        // not enough buffered yet, output silence
        for (let i=0; i<needed; i++) out[i] = 0;
        return true;
      }
      this.started = true;
    }

    // Fill output buffer by pulling & resampling from inBuffer
    for (let i = 0; i < needed; i++) {
      const srcPos = (this.outBuffer.length + i) * this.ratio + this.inIndex;
      const i0 = Math.floor(srcPos);
      const i1 = i0 + 1;
      const frac = srcPos - i0;
      const s0 = (i0 < this.inBuffer.length ? this.inBuffer[i0] : 0);
      const s1 = (i1 < this.inBuffer.length ? this.inBuffer[i1] : 0);
      // linear interpolate and normalize
      out[i] = ((s0 + (s1 - s0) * frac) / 32768);
    }

    // advance inIndex by however many source samples we consumed
    this.inIndex += Math.floor((needed) * this.ratio);

    // drop consumed samples from inBuffer
    if (this.inIndex > 0) {
      this.inBuffer = this.inBuffer.subarray(this.inIndex);
      this.inIndex = 0;
    }

    return true;
  }
}

registerProcessor('pcm-resampler', PCMResamplerProcessor);
