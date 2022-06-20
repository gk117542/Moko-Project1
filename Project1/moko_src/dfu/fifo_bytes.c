#include "fifo_bytes.h"

void queue_dt_init(STU_QUEUE *StuQueue,uint8_t *pbuf,uint16_t buf_len,uint8_t max_fram_len)
{
	StuQueue->max_fram_len=max_fram_len;
	StuQueue->dt_in=0;
	StuQueue->dt_out=0;
	StuQueue->dt_buf=pbuf;
	StuQueue->max_fram=buf_len/(max_fram_len+1);

}

/*
return : 
true: queue is empty
false: queue have data
*/

bool queue_is_empty(STU_QUEUE *StuQueue)
{
    if(StuQueue->dt_in==StuQueue->dt_out)
    return true;
    else
    return false;
}


/*
return : 
true: para is right
false: para is err,or queue full
*/

bool queue_dt_in(STU_QUEUE *StuQueue,uint8_t *datain,uint8_t len)
{   
    uint8_t *buf;
    bool ret=true;
    if(len>StuQueue->max_fram_len)// length is too long
    {
        len=StuQueue->max_fram_len;
        ret=false;
    }
    
    if(((StuQueue->dt_in+1)%StuQueue->max_fram)==StuQueue->dt_out)//queue is full
    {
         ret=false;
    }
    
    buf= (StuQueue->dt_buf)+(StuQueue->dt_in)*(StuQueue->max_fram_len+1);
    memcpy(&buf[1],datain,len);
    buf[0]=len;
    StuQueue->dt_in++;
    StuQueue->dt_in%=StuQueue->max_fram;
    
    return ret;


}



/*
para:   pdata--------->point out data buf
        data_len------>point out data length

return : 
true:have data in queue
false:no data in queue
*/



bool queue_dt_out(STU_QUEUE *StuQueue,uint8_t *pdata,uint8_t* data_len)
{
       
    uint8_t *buf;
    if(StuQueue->dt_in == StuQueue->dt_out) 
    {
        return false;
    }
    else
    {
        buf= (StuQueue->dt_buf)+(StuQueue->dt_out)*(StuQueue->max_fram_len+1);
        *data_len=buf[0];
        memcpy(pdata,&buf[1],*data_len);
        return true;
    }
}

/*
para:   cnt -->1 means deletes n frams
        
*/
void queue_dt_delete(STU_QUEUE *StuQueue,uint8_t cnt)
{
    uint16_t i,length;

    if(cnt == 0) return;

    length=cnt;

    for(i=0;i<length;i++)
    {

        if(StuQueue->dt_in == StuQueue->dt_out) 
        {
        return;
        }
        StuQueue->dt_out++;
        StuQueue->dt_out%=StuQueue->max_fram;

    }
}

