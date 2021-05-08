import './style.css'
import he from 'he';

/**
 * Wrapper for GLTF progress loader
 */
export class GLTFProgress {
    static POPUP_TIMEOUT = 3000;
    static MAX_LENGTH = 20;
    static progMsgs = {};

    /**
     * Updates GLTF Progress
     * @param {boolean} failed whether or not download was successful
     * @param {object} evt gltf event
     */
    static updateProgress(failed, evt) {
        if (!failed) {
            this.progMsgs[evt.detail.src] = evt.detail.progress;
        } else {
            this.progMsgs[evt.detail.src] = 'failed';
        }

        const gltfProgressEl = document.getElementById('gltf-loading');
        let innerHTML = 'Loading 3D model:<br/>';
        for (const [src, progress] of Object.entries(this.progMsgs)) {
            if (progress === 'failed') {
                innerHTML += `<b>"${he.encode(src)}"<br/>Failed!</b><br/>`;
            } else {
                let progessStr = '';
                if (evt.detail.progress !== Infinity) progessStr = `<br/>${parseFloat(progress.toFixed(1))}%<br/>`
                const shortName = src.length < this.MAX_LENGTH ?
                                    src : `…${src.substring(src.length - this.MAX_LENGTH)}`;
                innerHTML += `${he.encode(shortName)}${progessStr}`;
            }
        }

        gltfProgressEl.innerHTML = innerHTML;
        gltfProgressEl.className = 'show';
        if (evt.detail.progress === 100 || evt.detail.progress === Infinity || failed) {
            setTimeout(() => {
                this.progMsgs = {};
                gltfProgressEl.className = 'hide';
            }, this.POPUP_TIMEOUT);
        }

        if (evt.detail.progress === 100) {
            delete this.progMsgs[evt.detail.src];
        }
    };
}
