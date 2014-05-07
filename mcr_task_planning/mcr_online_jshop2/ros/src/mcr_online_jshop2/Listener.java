package mcr_online_jshop2;

import org.ros.message.MessageListener;

public class Listener<T> implements MessageListener<T> {
    private T data = null;

    public Listener() {
    }

    public void setData(T d) {
        data = d;
    }

    public boolean hasData() {
        return (data != null);
    }

    public T consumeData() {
        T temp = data;
        data = null;
        
        return temp;
    }

    public void onNewMessage(T msg) {
        setData(msg);
    }

}