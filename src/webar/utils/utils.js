export class Utils {
    static isMobile () {
        return /Android|mobile|iPad|iPhone/i.test(navigator.userAgent);
    }

    static dec2bin(dec) {
        return (dec >>> 0).toString(2);
    }

    static round3(num) {
        return Math.round((num + Number.EPSILON) * 1000) / 1000;
    }
}
