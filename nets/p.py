import unittest
import time
# STP is a protocol for sharing dictionary values.
# Some keys are reserved by "convention":
# _n where in is a number returns a key name
# __DATE__ contains the date the endpoint firmware was built
# __HELP__ contains human readable help lable
# __CONFIG_STATE__ Is a list of names that make up the configuration state vector
# __PROCES_STATE__ Is a list of names that make up the process state
# __REPORT_PROC__ requests the process state vector to be reported every so many "cycles" 0 is never and 1 is always
# __NAME__ The name of the device
# host is a transparent argument that can be used with the internet.
RESET = "+"
DELIM = " "
MTERM = "\n"


class stlendpoint:
    def __init__(self, name, d={}, tx=lambda x, y: print(x), names=[]):
        self.d = d
        self.tx = tx
        self.ournames = [name]+names
        self.eh = []
        self.ed = []
        self.heartbeat = None
        self.reporttime = 0

    def send(self, k, v, host):
        self.tx((RESET) + (k) + (DELIM) + (str(v)) + (MTERM), host)

    def query(self, k):
        self.tx((RESET) + (k) + (MTERM), host)

    def onset(self, f):
        self.eh = self.eh+[f]

    def ondname(self, f):
        self.ed = self.ed+[f]

    def set(self, k, v):
        if(v is None):
            for f in self.ed:
                f(k)
        else:
            for f in self.eh:
                f(k, v)
            self.d[k] = v

    def get(self, k):
        return d[k]

    def recl(self, line, host):
        self.heartbeat = time.time()
        k = ""
        v = ""
        name = ""
        namei = 0
        state = "lambda"
        for s in line:
            if(s == RESET):
                state = "lambda"
            if(state == "lambda"):
                k = ""
                v = ""
                namei = 0
                state = "k"
            elif(state == "k"):
                if(s == RESET):
                    state = "lambda"
                    next
                elif(s == DELIM):
                    k = k.rstrip()
                    state = "v"
                    next
                elif(s == MTERM):
                    k = k.rstrip()
                    if(len(k) == 0):
                        state = "lambda"
                        next
                    else:
                        if(k[0] == "_" and not k[1] == "_"):
                            self.tx(list(self.d)[
                                    int(k[len("_"):])], host)  # XXX
                            state = "lambda"
                            next
                        else:
                            self.send(k, self.get(k), host)
                            state = "lambda"
                            next
                else:
                    if(s.isalnum() or s == "_"):
                        if(len(k) > 40):
                            state = "w"
                            next
                        else:
                            k = k+s
                    else:
                        state = "w"
                        next
            elif(state == "v"):
                if(s == MTERM):
                    v = v.rstrip()
                    if(k[0] == "_" and not k[1] == "_"):
                        if(v not in self.d.keys()):
                            self.set(v, None)
                    else:
                        self.set(k, v)
                else:
                    if(len(v) > 40):
                        state = "w"
                        next
                    v = v+s
                    next
            elif(state == "w"):
                if(s == MTERM or s == RESET):
                    state = "lambda"

    def report(self, host):
        self.reporttime = self.reporttime+1
        if(self.reporttime > int(self.d["__REPORT_PERIOD__"])):
            self.reporttime = 0
            for i in self.d["__PROCESS_STATE__"].split(" "):
                self.send(i, self.d[i], host)


class stlendpointTest(unittest.TestCase):
    def test_query(self):
        s = ""
        tn = "testname"
        def ts(x): nonlocal s
        s = s+x
        t = stlendpoint("t", tx=ts)
        t.query(tn)
        self.assertEqual(s, "+"+tn+"\n")

    def test_send(self):
        s = ""
        tn = "testname"
        tv = 5.5
        def ts(x): nonlocal s
        s = s+x
        t = stlendpoint("t", tx=ts)
        t.send(tn, tv)
        self.assertEqual(s, "+"+tn+" "+str(tv)+"\n")

    def test_reclq(self):
        s = ""
        tn = "testname"
        tv = 5.5
        def ts(x): nonlocal s
        s = s+x
        t = stlendpoint("t", tx=ts, d={tn: tv})
        t.recl("+"+tn+"\n")
        self.assertEqual(s, "+"+tn+" "+str(tv)+"\n")

    def test_recls(self):
        s = ""
        tn = "testname"
        tv = 5.5
        tvn = 6.6
        def ts(x): nonlocal s
        s = s+x
        t = stlendpoint("t", tx=ts, d={tn: tv})
        t.recl("+"+tn+" "+str(tvn)+"\n")
        self.assertEqual(t.d[tn], str(tvn))

    def test_upev(self):
        tn = "testname"
        tv = 5.5
        tvn = 6.6
        tehcalled = False
        def teh(k, v): nonlocal tehcalled
        tehcalled = True
        t = stlendpoint("t", tx=ts, d={tn: tv})
        t.onset(teh)
        t.recl("+"+tn+" "+str(tvn)+"\n")
        self.assertEqual(tehcalled, False)
    # TODO: test names
