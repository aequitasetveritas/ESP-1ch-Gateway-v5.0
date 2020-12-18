import React, { Component, Fragment } from 'react';

import { withStyles } from '@material-ui/core/styles';
import Button from '@material-ui/core/Button';
import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import ListItemText from '@material-ui/core/ListItemText';
import ListItemAvatar from '@material-ui/core/ListItemAvatar'
import Avatar from '@material-ui/core/Avatar';
import Divider from '@material-ui/core/Divider';
import WifiIcon from '@material-ui/icons/LeakAdd';
import ArrowUpwardIcon from '@material-ui/icons/ArrowUpward';
import ArrowDownwardIcon from '@material-ui/icons/ArrowDownward';

import Table from '@material-ui/core/Table';
import TableHead from '@material-ui/core/TableHead';
import TableCell from '@material-ui/core/TableCell';
import TableBody from '@material-ui/core/TableBody';
import TableRow from '@material-ui/core/TableRow';
import Typography from '@material-ui/core/Typography';
import RefreshIcon from '@material-ui/icons/Refresh';

import SectionContent from '../components/SectionContent';
import * as Highlight from '../constants/Highlight';
import { restComponent } from '../components/RestComponent';
import LoadingNotification from '../components/LoadingNotification';

import { ENDPOINT_ROOT } from '../constants/Env';

const styles = theme => ({
  ["wifiStatus_" + Highlight.IDLE]: {
    backgroundColor: theme.palette.highlight_idle
  },
  ["wifiStatus_" + Highlight.SUCCESS]: {
    backgroundColor: theme.palette.highlight_success
  },
  ["wifiStatus_" + Highlight.ERROR]: {
    backgroundColor: theme.palette.highlight_error
  },
  ["wifiStatus_" + Highlight.WARN]: {
    backgroundColor: theme.palette.highlight_warn
  },
  button: {
    marginRight: theme.spacing(2),
    marginTop: theme.spacing(2),
  }
});

class LoraTabStatus extends Component {
  componentDidMount() {
    this.props.loadData();
  }


  createListItems(data, classes) {
    return (
      <Fragment>
 
        <ListItem>
          <ListItemAvatar>
            <Avatar className={classes["wifiStatus_" + (data.connBroker ? Highlight.SUCCESS : Highlight.ERROR) ]}>
              <WifiIcon />
            </Avatar>
          </ListItemAvatar>
          <ListItemText primary="Conexión al broker" secondary={data.connBroker ? "Conectado" : "No Conectado"} />
        </ListItem>
        <ListItem>
          <ListItemAvatar>
            <Avatar className={classes["wifiStatus_" + (Highlight.SUCCESS) ]}>
              <ArrowUpwardIcon />
            </Avatar>
          </ListItemAvatar>
          <ListItemText primary="UpPackets" secondary={data.up} />
        </ListItem>
        <ListItem>
          <ListItemAvatar>
            <Avatar className={classes["wifiStatus_" + (Highlight.SUCCESS) ]}>
              <ArrowDownwardIcon />
            </Avatar>
          </ListItemAvatar>
          <ListItemText primary="DownPackets" secondary={data.dw} />
        </ListItem>
        <Divider variant="inset" component="li" />
      </Fragment>
    );
  }

  renderGprsStatus(data, classes) {
    return (
      <div>
        <List>
          {this.createListItems(data, classes)}
        </List>
        <Button startIcon={<RefreshIcon />} variant="contained" color="secondary" className={classes.button} onClick={this.props.loadData}>
          Refresh
        </Button>
      </div>
    );
  }

  render() {
    const { data, fetched, errorMessage, loadData, classes } = this.props;
    return (
      <SectionContent title="Lora Estado">
        <LoadingNotification
          onReset={loadData}
          fetched={fetched}
          errorMessage={errorMessage}
          render={
            () => this.renderGprsStatus(data, classes)
          }
        />
      </SectionContent>
    );
  }

}

export default restComponent(ENDPOINT_ROOT + "gprsStatus",withStyles(styles)(LoraTabStatus));
